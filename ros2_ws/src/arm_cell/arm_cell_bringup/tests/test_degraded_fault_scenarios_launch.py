"""Verify configured communication degradation through the VDA and Safety owners."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from arm_cell_interfaces.msg import MotionCapability, MotionStatus, SafetyState
from std_srvs.srv import SetBool


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
                Node(
                    package="arm_cell_safety",
                    executable="safety_node",
                    name="safety_node",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": False,
                            "freshness_timeout_ms": 500,
                            "degraded_freshness_threshold_ms": 180,
                            "degraded_freshness_inputs": ["AMR", "PACKML"],
                            "degraded_velocity_scale": 0.5,
                            "degraded_acceleration_scale": 0.6,
                        }
                    ],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestDegradedFaultScenarios(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "degraded_fault_scenarios_test", use_global_arguments=False
        )
        self.states = []
        self.node.create_subscription(
            SafetyState, "/safety/state", self.states.append, 10
        )
        self.motion_publisher = self.node.create_publisher(
            MotionStatus, "/motion/state", 10
        )
        self.node.create_timer(0.05, self.publish_motion)
        self.degradation_client = self.node.create_client(
            SetBool, "/external_state_mock/fault/communication_degradation"
        )
        self.loss_client = self.node.create_client(
            SetBool, "/external_state_mock/fault/communication_loss"
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def publish_motion(self):
        message = MotionStatus()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.execution_state = MotionStatus.MOTION_STATE_IDLE
        message.backend_inactivity_confirmed = True
        self.motion_publisher.publish(message)

    def spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def set_fault(self, client, active):
        self.spin_until(client.service_is_ready)
        request = SetBool.Request()
        request.data = active
        future = client.call_async(request)
        self.spin_until(future.done)
        self.assertTrue(future.result().success)

    def test_degradation_restricts_then_restores_and_loss_fails_closed(self):
        self.spin_until(
            lambda: self.states
            and self.states[-1].required_inputs_fresh
            and self.states[-1].motion_capability.value
            == MotionCapability.MOTION_NORMAL
        )
        normal = self.states[-1]
        self.assertTrue(normal.motion_envelope_valid)
        self.assertAlmostEqual(normal.max_velocity_scale, 1.0)
        self.assertAlmostEqual(normal.max_acceleration_scale, 1.0)

        self.set_fault(self.degradation_client, True)
        self.spin_until(
            lambda: self.states
            and self.states[-1].required_inputs_fresh
            and self.states[-1].motion_envelope_valid
            and self.states[-1].max_velocity_scale == 0.5,
            timeout=2.0,
        )
        degraded = self.states[-1]
        self.assertEqual(degraded.safety_state, SafetyState.SAFETY_STATE_SAFE)
        self.assertEqual(
            degraded.motion_capability.value, MotionCapability.MOTION_NORMAL
        )
        self.assertAlmostEqual(degraded.max_acceleration_scale, 0.6)

        self.set_fault(self.degradation_client, False)
        self.spin_until(
            lambda: self.states
            and self.states[-1].required_inputs_fresh
            and self.states[-1].motion_envelope_valid
            and self.states[-1].max_velocity_scale == 1.0,
            timeout=1.0,
        )

        self.set_fault(self.loss_client, True)
        self.spin_until(
            lambda: self.states
            and not self.states[-1].required_inputs_fresh
            and not self.states[-1].motion_envelope_valid
            and self.states[-1].motion_capability.value == MotionCapability.MOTION_NONE,
            timeout=2.0,
        )
