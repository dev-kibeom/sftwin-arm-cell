"""ROS-level publication and mock fault-control checks."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import pytest
import rclpy
from arm_cell_interfaces.msg import (
    AMRDockingState,
    HandoffTrigger,
    MaterialHandoffState,
    PackMLState,
    SafetyHardwareState,
)
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    external_state_mock = Node(
        package="arm_cell_vda",
        executable="external_state_mock_node",
        name="external_state_mock",
        output="screen",
        parameters=[{"degraded_publish_period_ms": 300}],
    )
    return (
        launch.LaunchDescription(
            [
                external_state_mock,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"external_state_mock": external_state_mock},
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
        self.material_handoff = []
        self.node.create_subscription(
            MaterialHandoffState,
            "/vda/material_handoff_state",
            self.material_handoff.append,
            10,
        )
        self.accepted_handoff_publisher = self.node.create_publisher(
            HandoffTrigger, "/integration/accepted_handoff", 10
        )
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

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
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

    def call_fault(self, name, active):
        client = self.node.create_client(SetBool, f"/external_state_mock/fault/{name}")
        self.spin_until(client.service_is_ready)
        request = SetBool.Request()
        request.data = active
        future = client.call_async(request)
        self.spin_until(future.done)
        response = future.result()
        self.node.destroy_client(client)
        return response

    def test_safety_signal_loss_suppresses_only_safety_hardware_for_five_seconds(self):
        self.spin_until(lambda: self.amr and self.packml and self.hardware)
        self.hardware.clear()
        self.amr.clear()
        self.packml.clear()
        self.assertTrue(self.call_fault("safety_signal_loss", True).success)
        self.assertFalse(self.call_fault("safety_signal_loss", True).success)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(self.amr)
        self.assertTrue(self.packml)
        self.assertFalse(self.hardware)
        self.spin_until(lambda: self.hardware, timeout=5.5)
        recovered_at = time.monotonic()
        self.assertTrue(self.hardware[-1].valid)
        self.assertGreater(recovered_at, deadline)

    def test_safety_signal_loss_cancel_publishes_fresh_hardware_immediately(self):
        self.spin_until(lambda: self.hardware)
        self.hardware.clear()
        self.assertTrue(self.call_fault("safety_signal_loss", True).success)
        self.hardware.clear()
        self.assertTrue(self.call_fault("safety_signal_loss", False).success)
        self.spin_until(lambda: self.hardware, timeout=0.5)

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

    def test_material_delivery_starts_only_from_valid_accepted_trigger(
        self, external_state_mock, proc_output
    ):
        services = dict(self.node.get_service_names_and_types())
        self.assertNotIn("/vda/request_material", services)
        self.spin_until(
            lambda: self.node.count_subscribers("/integration/accepted_handoff") > 0
        )

        trigger = HandoffTrigger()
        trigger.delivery_id.uuid[0] = 42
        trigger.quantity = 1
        trigger.material_type = "RawPart"
        trigger.valid = False
        self.accepted_handoff_publisher.publish(trigger)
        quiet_deadline = time.monotonic() + 0.25
        while time.monotonic() < quiet_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertFalse(self.material_handoff)

        trigger.valid = True
        self.accepted_handoff_publisher.publish(trigger)
        self.spin_until(
            lambda: any(
                message.phase == MaterialHandoffState.HANDOFF_DEPARTED
                for message in self.material_handoff
            )
        )
        phases = [message.phase for message in self.material_handoff]
        self.assertIn(MaterialHandoffState.HANDOFF_ARRIVED, phases)
        self.assertIn(MaterialHandoffState.HANDOFF_DOCKED, phases)
        self.assertIn(MaterialHandoffState.HANDOFF_UNLOADED, phases)
        self.assertIn(MaterialHandoffState.HANDOFF_DEPARTED, phases)
        self.assertTrue(
            all(
                tuple(message.delivery_id.uuid) == tuple(trigger.delivery_id.uuid)
                for message in self.material_handoff
            )
        )
        proc_output.assertWaitFor(
            "material delivery phase changed phase=ARRIVED",
            process=external_state_mock,
        )
        proc_output.assertWaitFor(
            "material delivery phase changed phase=DEPARTED",
            process=external_state_mock,
        )

    def test_material_delivery_failure_is_logged_once(
        self, external_state_mock, proc_output
    ):
        self.spin_until(
            lambda: self.node.count_subscribers("/integration/accepted_handoff") > 0
        )
        trigger = HandoffTrigger()
        trigger.delivery_id.uuid[0] = 43
        trigger.quantity = 1
        trigger.material_type = "RawPart"
        trigger.valid = True
        self.accepted_handoff_publisher.publish(trigger)
        self.spin_until(
            lambda: any(
                message.delivery_id == trigger.delivery_id
                for message in self.material_handoff
            )
        )
        self.set_fault("material_handoff_failure", True)
        proc_output.assertWaitFor(
            "material delivery phase failed phase=FAILED",
            process=external_state_mock,
        )
        self.spin_for(0.25)
        self.assertEqual(
            sum(
                message.phase == MaterialHandoffState.HANDOFF_FAILED
                for message in self.material_handoff
            ),
            1,
        )
