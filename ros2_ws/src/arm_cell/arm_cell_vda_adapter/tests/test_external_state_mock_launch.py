"""ROS-level publication and mock fault-control checks."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import pytest
import rclpy
from arm_cell_interfaces.msg import AMRDockingState, PackMLState, SafetyHardwareState
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                Node(
                    package="arm_cell_vda",
                    executable="external_state_mock_node",
                    name="external_state_mock",
                    output="screen",
                    parameters=[{"degraded_publish_period_ms": 300}],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestExternalStateMockLaunch(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "external_state_mock_test", use_global_arguments=False
        )
        self.amr = []
        self.packml = []
        self.hardware = []
        self.node.create_subscription(
            AMRDockingState, "/amr/docking_report", self.amr.append, 10
        )
        self.node.create_subscription(
            PackMLState, "/packml/state", self.packml.append, 10
        )
        self.node.create_subscription(
            SafetyHardwareState, "/safety/hardware_state", self.hardware.append, 10
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def set_fault(self, name, active):
        client = self.node.create_client(SetBool, f"/external_state_mock/fault/{name}")
        self.spin_until(client.service_is_ready)
        request = SetBool.Request()
        request.data = active
        future = client.call_async(request)
        self.spin_until(future.done)
        self.assertTrue(future.result().success)
        self.node.destroy_client(client)

    def test_communication_loss_silences_and_restores_all_topics(self):
        self.spin_until(lambda: self.amr and self.packml and self.hardware)
        self.amr.clear()
        self.packml.clear()
        self.hardware.clear()

        self.set_fault("communication_loss", True)
        self.amr.clear()
        self.packml.clear()
        self.hardware.clear()
        silence_deadline = time.monotonic() + 0.35
        while time.monotonic() < silence_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertFalse(self.amr)
        self.assertFalse(self.packml)
        self.assertFalse(self.hardware)

        self.set_fault("communication_loss", False)
        self.spin_until(lambda: self.amr and self.packml and self.hardware)
        self.assertGreaterEqual(self.amr[-1].header.stamp.sec, 0)
        self.assertTrue(self.amr[-1].valid)

    def test_clear_undock_does_not_synthesize_redock(self):
        self.set_fault("premature_undock", True)
        self.spin_until(
            lambda: self.amr
            and self.amr[-1].docking_state == AMRDockingState.AMR_DOCKING_UNDOCKED
        )
        self.assertTrue(self.amr[-1].driving)

        self.set_fault("premature_undock", False)
        self.amr.clear()
        self.spin_until(lambda: self.amr)
        self.assertTrue(
            all(
                message.docking_state == AMRDockingState.AMR_DOCKING_UNDOCKED
                and message.driving
                for message in self.amr
            )
        )

    def test_communication_degradation_is_bounded_and_clears_promptly(self):
        self.spin_until(lambda: self.amr)
        baseline_stamp = (
            self.amr[-1].header.stamp.sec + self.amr[-1].header.stamp.nanosec / 1e9
        )
        self.set_fault("communication_degradation", True)
        self.amr.clear()
        self.spin_until(
            lambda: self.amr
            and self.amr[-1].header.stamp.sec
            + self.amr[-1].header.stamp.nanosec / 1e9
            - baseline_stamp
            >= 0.25,
            timeout=2.0,
        )
        degraded_stamp = (
            self.amr[-1].header.stamp.sec + self.amr[-1].header.stamp.nanosec / 1e9
        )
        self.assertLess(degraded_stamp - baseline_stamp, 0.5)

        self.amr.clear()
        self.set_fault("communication_degradation", False)
        self.spin_until(lambda: self.amr, timeout=0.2)
        self.assertTrue(self.amr[-1].valid)
