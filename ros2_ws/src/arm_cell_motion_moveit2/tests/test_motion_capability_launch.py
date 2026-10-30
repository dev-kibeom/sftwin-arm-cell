"""ROS-level SafetyState to Motion action permission checks."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import pytest
import rclpy
from arm_cell_interfaces.action import ExecuteTask
from arm_cell_interfaces.msg import MotionCapability, MotionTaskType, SafetyState
from rclpy.action import ActionClient


@pytest.mark.launch_test
def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                Node(
                    package="arm_cell_motion_moveit2",
                    executable="motion_node",
                    name="motion_node",
                    parameters=[
                        {
                            "backend_available": True,
                            "backend_availability_delay_ms": 300,
                        }
                    ],
                    output="screen",
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestMotionCapabilityLaunch(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "motion_capability_test", use_global_arguments=False
        )
        self.safety_publisher = self.node.create_publisher(
            SafetyState, "/safety/state", 10
        )
        self.action_client = ActionClient(
            self.node, ExecuteTask, "/motion/execute_task"
        )

    def tearDown(self):
        self.action_client.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish_capability(self, capability):
        self.spin_until(lambda: self.safety_publisher.get_subscription_count() > 0)
        message = SafetyState()
        message.safety_state = SafetyState.SAFETY_STATE_SAFE
        message.motion_capability.value = capability
        message.valid = True
        message.required_inputs_fresh = True
        message.motion_envelope_valid = True
        message.max_velocity_scale = 1.0
        message.max_acceleration_scale = 1.0
        for _ in range(10):
            self.safety_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def send_goal(self, task_type):
        goal = ExecuteTask.Goal()
        goal.task_type.value = task_type
        future = self.action_client.send_goal_async(goal)
        self.spin_until(future.done)
        return future.result()

    def test_safety_capability_controls_motion_action_path(self):
        self.spin_until(self.action_client.server_is_ready)

        # No SafetyState has reached Motion yet: the action is fail-closed.
        rejected = self.send_goal(MotionTaskType.TASK_TYPE_GO_HOME)
        self.assertFalse(rejected.accepted)

        self.publish_capability(MotionCapability.MOTION_NORMAL)
        normal_goal = self.send_goal(MotionTaskType.TASK_TYPE_GO_HOME)
        self.assertTrue(normal_goal.accepted)
        normal_result = normal_goal.get_result_async()
        self.spin_until(normal_result.done)
        self.assertEqual(
            normal_result.result().result.result_code.value,
            9,  # TASK_RESULT_BACKEND_ERROR: NORMAL passed Motion's gate.
        )

        self.publish_capability(MotionCapability.MOTION_RECOVERY_ONLY)
        rejected_normal = self.send_goal(MotionTaskType.TASK_TYPE_GO_HOME)
        self.assertFalse(rejected_normal.accepted)
        accepted_retract = self.send_goal(MotionTaskType.TASK_TYPE_RETRACT)
        self.assertTrue(accepted_retract.accepted)
        retract_result = accepted_retract.get_result_async()
        self.spin_until(retract_result.done)
        self.assertEqual(retract_result.result().result.result_code.value, 9)

        self.publish_capability(MotionCapability.MOTION_NORMAL)
        accepted_before_downgrade = self.send_goal(MotionTaskType.TASK_TYPE_GO_HOME)
        self.assertTrue(accepted_before_downgrade.accepted)
        self.publish_capability(MotionCapability.MOTION_NONE)
        downgraded_result = accepted_before_downgrade.get_result_async()
        self.spin_until(downgraded_result.done)
        self.assertEqual(
            downgraded_result.result().result.result_code.value,
            8,  # TASK_RESULT_PERMISSION_DENIED: latest capability blocked submit.
        )
