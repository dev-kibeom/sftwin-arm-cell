"""Verify the real SafetyNode -> MotionNode direct stop path."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import pytest
import rclpy
from arm_cell_interfaces.action import ExecuteTask
from arm_cell_interfaces.msg import AMRDockingState
from arm_cell_interfaces.msg import MotionStatus
from arm_cell_interfaces.msg import MotionTaskType
from arm_cell_interfaces.msg import PackMLState
from arm_cell_interfaces.msg import SafetyHardwareState
from arm_cell_interfaces.msg import SafetyState
from arm_cell_interfaces.msg import StopMode
from rclpy.action import ActionClient
from rclpy.node import Node as RclpyNode
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                Node(
                    package="arm_cell_motion",
                    executable="motion_node",
                    name="motion_node",
                    parameters=[
                        {
                            "backend_available": True,
                            "enable_test_backend_controls": True,
                            "planning_velocity_scaling_factor": 0.1,
                            "planning_acceleration_scaling_factor": 0.1,
                        }
                    ],
                    output="screen",
                ),
                Node(
                    package="arm_cell_safety",
                    executable="safety_node",
                    name="safety_node",
                    output="screen",
                    parameters=[
                        {
                            "freshness_timeout_ms": 2000,
                        }
                    ],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestSafetyMotionStopIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = RclpyNode("safety_motion_stop_integration")
        self.amr_publisher = self.node.create_publisher(
            AMRDockingState, "/amr/docking_report", 10
        )
        self.packml_publisher = self.node.create_publisher(
            PackMLState, "/packml/state", 10
        )
        self.hardware_publisher = self.node.create_publisher(
            SafetyHardwareState, "/safety/hardware_state", 10
        )
        self.motion_statuses = []
        self.safety_states = []
        self.stop_modes = []
        self.node.create_subscription(
            MotionStatus, "/motion/state", self.motion_statuses.append, 10
        )
        self.node.create_subscription(
            SafetyState, "/safety/state", self.safety_states.append, 10
        )
        self.node.create_subscription(
            StopMode,
            "/motion/test_backend/stop_mode",
            self.stop_modes.append,
            10,
        )
        self.action_client = ActionClient(
            self.node, ExecuteTask, "/motion/execute_task"
        )
        self.active_client = self.node.create_client(
            SetBool, "/motion/test_backend/execution_active"
        )
        self.confirmed_client = self.node.create_client(
            SetBool, "/motion/test_backend/inactivity_confirmed"
        )
        self.goal_handle = None
        self.hardware_state = SafetyHardwareState()
        self.hardware_state.valid = True
        self.node.create_timer(0.05, self.publish_hardware)

    def tearDown(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            latest_safety = self.safety_states[-1] if self.safety_states else None
            latest_motion = self.motion_statuses[-1] if self.motion_statuses else None
            self.assertLess(
                time.monotonic(),
                deadline,
                "safety=%r motion=%r" % (latest_safety, latest_motion),
            )
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish_inputs(self, docking_state, packml_state, e_stop=False):
        amr = AMRDockingState()
        amr.docking_state = docking_state
        amr.valid = True
        packml = PackMLState()
        packml.state = packml_state
        packml.valid = True
        hardware = SafetyHardwareState()
        hardware.e_stop_active = e_stop
        hardware.valid = True
        hardware.header.stamp = self.node.get_clock().now().to_msg()
        self.hardware_state = hardware
        for _ in range(4):
            self.amr_publisher.publish(amr)
            self.packml_publisher.publish(packml)
            self.hardware_publisher.publish(hardware)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish_hardware(self):
        self.hardware_state.header.stamp = self.node.get_clock().now().to_msg()
        self.hardware_publisher.publish(self.hardware_state)

    def establish_safe_state(self):
        self.spin_until(
            lambda: self.hardware_publisher.get_subscription_count() > 0
            and self.node.count_publishers("/safety/state") > 0
            and self.node.count_subscribers("/safety/state") >= 2
            and self.node.count_subscribers("/motion/state") > 0
        )
        deadline = time.monotonic() + 8.0
        while not self.latest_safety_state(SafetyState.SAFETY_STATE_SAFE):
            self.publish_inputs(
                AMRDockingState.AMR_DOCKING_DOCKED,
                PackMLState.PACKML_STATE_IDLE,
            )
            latest_safety = self.safety_states[-1] if self.safety_states else None
            latest_motion = self.motion_statuses[-1] if self.motion_statuses else None
            self.assertLess(
                time.monotonic(),
                deadline,
                "safety=%r motion=%r" % (latest_safety, latest_motion),
            )
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def send_goal_after_safe_state(self):
        deadline = time.monotonic() + 8.0
        while True:
            goal = ExecuteTask.Goal()
            goal.task_type.value = MotionTaskType.TASK_TYPE_GO_HOME
            goal_future = self.action_client.send_goal_async(goal)
            while not goal_future.done():
                self.assertLess(time.monotonic(), deadline)
                rclpy.spin_once(self.node, timeout_sec=0.02)
            goal_handle = goal_future.result()
            if goal_handle.accepted:
                return goal_handle
            self.publish_inputs(
                AMRDockingState.AMR_DOCKING_DOCKED,
                PackMLState.PACKML_STATE_IDLE,
            )
            self.assertLess(time.monotonic(), deadline)

    def send_goal_without_refreshing_safety_inputs(self):
        goal = ExecuteTask.Goal()
        goal.task_type.value = MotionTaskType.TASK_TYPE_GO_HOME
        future = self.action_client.send_goal_async(goal)
        self.spin_until(future.done)
        goal_handle = future.result()
        latest_safety = self.safety_states[-1] if self.safety_states else None
        latest_motion = self.motion_statuses[-1] if self.motion_statuses else None
        self.assertTrue(
            goal_handle.accepted,
            "safety=%r motion=%r" % (latest_safety, latest_motion),
        )
        return goal_handle

    def set_backend_state(self, client, value):
        self.spin_until(client.service_is_ready)
        future = client.call_async(SetBool.Request(data=value))
        self.spin_until(future.done)
        self.assertTrue(future.result().success)

    def latest_motion_state(self, value):
        return (
            bool(self.motion_statuses)
            and self.motion_statuses[-1].execution_state == value
        )

    def latest_safety_state(self, value):
        return bool(self.safety_states) and self.safety_states[-1].safety_state == value

    def test_external_state_does_not_interlock_and_estop_stops_directly(self):
        """External state is ignored; E-Stop retains the direct Safety stop path."""
        self.spin_until(self.action_client.server_is_ready)
        self.establish_safe_state()

        self.goal_handle = self.send_goal_after_safe_state()
        self.spin_until(
            lambda: self.latest_motion_state(MotionStatus.MOTION_STATE_EXECUTING)
        )
        stop_count = len(self.stop_modes)
        self.publish_inputs(
            AMRDockingState.AMR_DOCKING_UNDOCKED,
            PackMLState.PACKML_STATE_ABORTED,
        )
        for _ in range(15):
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(self.latest_motion_state(MotionStatus.MOTION_STATE_EXECUTING))
        self.assertEqual(len(self.stop_modes), stop_count)
        self.assertEqual(self.safety_states[-1].active_causes.value, 0)
        self.assertEqual(self.safety_states[-1].latched_causes.value, 0)

        self.publish_inputs(
            AMRDockingState.AMR_DOCKING_UNDOCKED,
            PackMLState.PACKML_STATE_ABORTED,
            e_stop=True,
        )
        self.spin_until(
            lambda: len(self.stop_modes) > stop_count
            and self.stop_modes[-1].value == StopMode.STOP_MODE_EMERGENCY
        )
        self.spin_until(
            lambda: self.latest_motion_state(MotionStatus.MOTION_STATE_STOPPING)
        )
        self.assertEqual(
            self.safety_states[-1].safety_state,
            SafetyState.SAFETY_STATE_EMERGENCY_STOPPED,
        )

        self.set_backend_state(self.active_client, False)
        self.set_backend_state(self.confirmed_client, True)
        self.spin_until(
            lambda: self.latest_motion_state(MotionStatus.MOTION_STATE_STOPPED)
        )

        self.publish_inputs(
            AMRDockingState.AMR_DOCKING_DOCKED,
            PackMLState.PACKML_STATE_IDLE,
        )
        deadline = time.monotonic() + 8.0
        while not (
            self.safety_states
            and self.safety_states[-1].safety_state
            == SafetyState.SAFETY_STATE_RECOVERY_REQUIRED
        ):
            self.publish_inputs(
                AMRDockingState.AMR_DOCKING_DOCKED,
                PackMLState.PACKML_STATE_IDLE,
            )
            latest_safety = self.safety_states[-1] if self.safety_states else None
            latest_motion = self.motion_statuses[-1] if self.motion_statuses else None
            self.assertLess(
                time.monotonic(),
                deadline,
                "safety=%r motion=%r" % (latest_safety, latest_motion),
            )
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(
            self.safety_states[-1].safety_state,
            SafetyState.SAFETY_STATE_RECOVERY_REQUIRED,
        )
        self.assertEqual(self.safety_states[-1].active_causes.value, 0)
        self.assertNotEqual(self.safety_states[-1].latched_causes.value & 1, 0)
        self.assertEqual(
            self.safety_states[-1].motion_capability.value,
            0,  # E_STOP remains latched; recovery is not authorized.
        )
