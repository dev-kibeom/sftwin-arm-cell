"""Verify configured communication degradation through the VDA and Safety owners."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from arm_cell_interfaces.msg import (
    MotionCapability,
    MotionStatus,
    SafetyHardwareState,
    SafetyState,
)
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
        self.hardware_samples = []
        self.node.create_subscription(
            SafetyState, "/safety/state", self.states.append, 10
        )
        self.node.create_subscription(
            SafetyHardwareState,
            "/safety/hardware_state",
            lambda message: self.hardware_samples.append(
                (
                    time.monotonic(),
                    message.header.stamp.sec * 1_000_000_000
                    + message.header.stamp.nanosec,
                )
            ),
            10,
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

    def test_delay_does_not_restrict_and_required_input_loss_fails_closed(self):
        self.spin_until(
            lambda: self.states
            and self.states[-1].required_inputs_fresh
            and self.states[-1].motion_capability.value
            == MotionCapability.MOTION_NORMAL
            and len(self.hardware_samples) >= 2
        )
        normal = self.states[-1]
        self.assertTrue(normal.motion_envelope_valid)
        self.assertAlmostEqual(normal.max_velocity_scale, 1.0)
        self.assertAlmostEqual(normal.max_acceleration_scale, 1.0)

        previous_hardware_receipt = self.hardware_samples[-1][0]
        self.set_fault(self.degradation_client, True)
        self.spin_until(
            lambda: self.hardware_samples
            and self.hardware_samples[-1][0] > previous_hardware_receipt,
            timeout=2.0,
        )
        self.assertGreaterEqual(
            self.hardware_samples[-1][0] - previous_hardware_receipt, 0.2
        )
        delayed_hardware_stamp = self.hardware_samples[-1][1]
        state_count = len(self.states)
        self.spin_until(
            lambda: len(self.states) > state_count
            and self.states[-1].header.stamp.sec * 1_000_000_000
            + self.states[-1].header.stamp.nanosec
            >= delayed_hardware_stamp
            and self.states[-1].required_inputs_fresh,
            timeout=1.0,
        )
        delayed = self.states[-1]
        self.assertTrue(delayed.required_inputs_fresh)
        self.assertEqual(delayed.safety_state, SafetyState.SAFETY_STATE_SAFE)
        self.assertEqual(
            delayed.motion_capability.value, MotionCapability.MOTION_NORMAL
        )
        self.assertTrue(delayed.motion_envelope_valid)
        self.assertAlmostEqual(delayed.max_acceleration_scale, 1.0)
        self.assertEqual(delayed.active_causes.value, 0)

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
