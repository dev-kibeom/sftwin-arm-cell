"""ROS boundary coverage for Calibration and Mission admission ownership."""

import threading
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
from arm_cell_interfaces.action import ExecuteTask
from arm_cell_interfaces.msg import (
    MaterialReadiness,
    MotionCapability,
    MotionTaskResultCode,
    SafetyState,
    VisionCalibration,
)
from launch_ros.actions import Node
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    recipe_directory = str(
        Path(__file__).resolve().parents[1].parent
        / "arm_cell_bringup"
        / "config"
        / "recipes"
    )
    orchestration = Node(
        package="arm_cell_orchestration",
        executable="orchestration_node",
        parameters=[
            {
                "mission_target_id": "RawPart",
                "recipe_directory": recipe_directory,
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([orchestration, launch_testing.actions.ReadyToTest()]),
        {"orchestration": orchestration},
    )


class TestOrchestrationNodeAdmission(unittest.TestCase):
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.node = rclpy.create_node(
            "orchestration_admission_contract_test",
            use_global_arguments=False,
            context=self.context,
        )
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.calibration_entered = threading.Event()
        self.release_calibration = threading.Event()
        self.calibration_calls = 0
        self.calibration_lock = threading.Lock()
        self.motion_goal_entered = threading.Event()
        self.release_motion = threading.Event()
        self.motion_goal_count = 0

        self.calibration_server = self.node.create_service(
            Trigger, "/vision/calibrate", self.handle_calibration
        )
        self.motion_server = ActionServer(
            self.node,
            ExecuteTask,
            "/motion/execute_task",
            execute_callback=self.execute_motion,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
        )
        self.calibration_client = self.node.create_client(
            Trigger, "/orchestration/calibrate_vision"
        )
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.safety_publisher = self.node.create_publisher(
            SafetyState, "/safety/state", 10
        )
        self.readiness_publisher = self.node.create_publisher(
            MaterialReadiness, "/integration/material_readiness", qos
        )
        self.calibration_publisher = self.node.create_publisher(
            VisionCalibration, "/vision/calibration_status", qos
        )

    def tearDown(self):
        self.release_calibration.set()
        self.release_motion.set()
        callbacks_stopped = self.executor.shutdown(timeout_sec=2.0)
        self.executor_thread.join(timeout=2.0)
        spin_thread_stopped = not self.executor_thread.is_alive()
        self.motion_server.destroy()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.assertTrue(
            callbacks_stopped, "ROS callbacks did not stop during fixture cleanup"
        )
        self.assertTrue(spin_thread_stopped, "ROS executor spin thread did not stop")

    def handle_calibration(self, _request, response):
        with self.calibration_lock:
            self.calibration_calls += 1
            call_number = self.calibration_calls
        self.calibration_entered.set()
        if call_number == 2:
            self.release_calibration.wait(timeout=5.0)
        response.success = call_number != 2
        response.message = (
            "calibration artifact created" if response.success else "calibration failed"
        )
        return response

    def execute_motion(self, goal_handle):
        self.motion_goal_count += 1
        self.motion_goal_entered.set()
        self.release_motion.wait(timeout=5.0)
        result = ExecuteTask.Result()
        result.result_code.value = MotionTaskResultCode.TASK_RESULT_BUSY
        result.diagnostic_detail = "reservation remains active"
        result.object_disposition = ExecuteTask.Result.OBJECT_DISPOSITION_UNCHANGED
        goal_handle.abort()
        return result

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.01)

    def publish_inputs(self, calibration_state):
        safety = SafetyState()
        safety.valid = True
        safety.required_inputs_fresh = True
        safety.safety_state = SafetyState.SAFETY_STATE_SAFE
        safety.motion_capability.value = MotionCapability.MOTION_NORMAL
        safety.motion_envelope_valid = True
        safety.max_velocity_scale = 1.0
        safety.max_acceleration_scale = 1.0

        readiness = MaterialReadiness()
        readiness.delivery_id.uuid[15] = 17
        readiness.material_type = "RawPart"
        readiness.quantity = 1
        readiness.valid = True
        readiness.material_ready = True

        calibration = VisionCalibration()
        calibration.state = calibration_state
        for _ in range(5):
            self.safety_publisher.publish(safety)
            self.readiness_publisher.publish(readiness)
            self.calibration_publisher.publish(calibration)
            time.sleep(0.02)

    def test_calibration_claim_and_mission_exclude_each_other(
        self, orchestration, proc_output
    ):
        self.spin_until(self.calibration_client.service_is_ready)
        self.spin_until(
            lambda: self.node.count_subscribers("/safety/state") > 0
            and self.node.count_subscribers("/integration/material_readiness") > 0
            and self.node.count_subscribers("/vision/calibration_status") > 0
        )

        first_calibration = self.calibration_client.call_async(Trigger.Request())
        self.spin_until(first_calibration.done)
        self.assertTrue(first_calibration.result().success)
        self.assertEqual(
            first_calibration.result().message, "calibration artifact created"
        )

        self.publish_inputs(VisionCalibration.STATE_NEEDS_CALIBRATION)
        self.assertFalse(
            self.motion_goal_entered.wait(timeout=0.1),
            "Mission started while Vision reported that calibration was required",
        )

        self.calibration_entered.clear()
        request_future = self.calibration_client.call_async(Trigger.Request())
        self.spin_until(self.calibration_entered.is_set)
        self.publish_inputs(VisionCalibration.STATE_READY)
        self.assertFalse(
            self.motion_goal_entered.wait(timeout=0.1),
            "Mission started while the Calibration service held its admission claim",
        )

        self.release_calibration.set()
        self.spin_until(request_future.done)
        calibration_response = request_future.result()
        self.assertFalse(calibration_response.success)
        self.assertEqual(calibration_response.message, "calibration failed")
        self.spin_until(self.motion_goal_entered.is_set)

        rejected = self.calibration_client.call_async(Trigger.Request())
        self.spin_until(rejected.done)
        self.assertFalse(rejected.result().success)
        self.assertIn("active Mission", rejected.result().message)
        self.assertEqual(self.calibration_calls, 2)

        self.release_motion.set()
        proc_output.assertWaitFor(
            "TASK_RESULT_BUSY",
            process=orchestration,
        )
        proc_output.assertWaitFor(
            "MISSION_EXIT_MOTION_ERROR",
            process=orchestration,
        )
        self.assertEqual(self.motion_goal_count, 1)
