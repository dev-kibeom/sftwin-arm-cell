"""Verify concurrent external deliveries reach the real Orchestration node."""

import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from arm_cell_interfaces.action import ExecuteCycle
from arm_cell_interfaces.action._execute_cycle import ExecuteCycle_FeedbackMessage
from arm_cell_interfaces.msg import (
    HandoffTrigger,
    HandoffTriggerStatus,
    MaterialHandoffState,
    MaterialReadiness,
    SafetyState,
    TransferCompletion,
    VisionCalibration,
)
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


MISSION_TARGET = "RawPart"


def generate_test_description():
    recipe_directory = Path(__file__).resolve().parents[1] / "config" / "recipes"
    return (
        launch.LaunchDescription(
            [
                Node(
                    package="arm_cell_gateway",
                    executable="material_handoff_node",
                    name="material_handoff_node",
                    output="screen",
                ),
                Node(
                    package="arm_cell_orchestration",
                    executable="orchestration_node",
                    name="orchestration_node",
                    output="screen",
                    parameters=[
                        {
                            "mission_target_id": MISSION_TARGET,
                            "recipe_directory": str(recipe_directory),
                        }
                    ],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestExternalMultiDeliveryOrchestration(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "external_multi_delivery_orchestration_test", use_global_arguments=False
        )
        self.accepted = []
        self.trigger_status = []
        self.readiness = []
        self.cycle_goal_ids_by_delivery = {}
        self.cycle_goal_status = {}
        self.node.create_subscription(
            HandoffTrigger,
            "/integration/accepted_handoff",
            self.accepted.append,
            10,
        )
        self.node.create_subscription(
            HandoffTriggerStatus,
            "/integration/handoff_trigger_status",
            self.trigger_status.append,
            10,
        )
        self.node.create_subscription(
            MaterialReadiness,
            "/integration/material_readiness",
            self.readiness.append,
            64,
        )
        self.node.create_subscription(
            ExecuteCycle_FeedbackMessage,
            "/orchestration/execute_cycle/_action/feedback",
            self.record_cycle_feedback,
            10,
        )
        self.node.create_subscription(
            GoalStatusArray,
            "/orchestration/execute_cycle/_action/status",
            self.record_cycle_status,
            10,
        )
        self.safety_publisher = self.node.create_publisher(
            SafetyState, "/safety/state", 10
        )
        self.trigger_publisher = self.node.create_publisher(
            HandoffTrigger, "/external/handoff_trigger", 10
        )
        self.handoff_publisher = self.node.create_publisher(
            MaterialHandoffState, "/vda/material_handoff_state", 10
        )
        self.completion_publisher = self.node.create_publisher(
            TransferCompletion, "/material_supply/transfer_completion", 10
        )
        self.vision_calibration_publisher = self.node.create_publisher(
            VisionCalibration,
            "/vision/calibration_status",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.execute_client = ActionClient(
            self.node, ExecuteCycle, "/orchestration/execute_cycle"
        )
        self.safety_timer = self.node.create_timer(0.05, self.publish_safe_state)
        self.calibration_timer = self.node.create_timer(0.2, self.publish_vision_ready)

    def tearDown(self):
        self.safety_timer.cancel()
        self.calibration_timer.cancel()
        self.node.destroy_node()
        rclpy.shutdown()

    def publish_safe_state(self):
        state = SafetyState()
        state.safety_state = SafetyState.SAFETY_STATE_SAFE
        state.motion_capability.value = state.motion_capability.MOTION_NORMAL
        state.valid = True
        state.required_inputs_fresh = True
        state.motion_envelope_valid = True
        state.max_velocity_scale = 1.0
        state.max_acceleration_scale = 1.0
        self.safety_publisher.publish(state)

    def publish_vision_ready(self):
        calibration = VisionCalibration()
        calibration.state = VisionCalibration.STATE_READY
        self.vision_calibration_publisher.publish(calibration)

    def spin_until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_second_delivery_ready_while_first_is_in_progress(self):
        self.spin_until(self.execute_client.server_is_ready)
        self.spin_until(
            lambda: self.node.count_subscribers("/external/handoff_trigger") >= 1
        )
        self.spin_until(
            lambda: self.node.count_subscribers("/vda/material_handoff_state") >= 1
        )
        self.spin_until(
            lambda: self.node.count_subscribers("/vision/calibration_status") >= 1
        )
        self.publish_vision_ready()

        first = self.make_trigger(101, 2)
        second = self.make_trigger(102, 3)
        self.trigger_publisher.publish(first)
        self.trigger_publisher.publish(second)
        self.spin_until(lambda: len(self.accepted) == 2)
        self.spin_until(lambda: len(self.trigger_status) == 2)
        self.assertTrue(
            all(
                status.state == HandoffTriggerStatus.TRIGGER_ACCEPTED
                for status in self.trigger_status
            )
        )

        self.publish_phase(first, MaterialHandoffState.HANDOFF_ARRIVED)
        self.publish_phase(first, MaterialHandoffState.HANDOFF_DOCKED)
        for phase in (
            MaterialHandoffState.HANDOFF_ARRIVED,
            MaterialHandoffState.HANDOFF_DOCKED,
            MaterialHandoffState.HANDOFF_UNLOADED,
            MaterialHandoffState.HANDOFF_DEPARTED,
        ):
            self.publish_phase(second, phase)
        self.completion_publisher.publish(self.make_completion(second))
        second_id = tuple(second.delivery_id.uuid)
        self.spin_until(lambda: self.count_ready(second_id) == 1)
        self.spin_until(lambda: self.count_cycle_goals(second_id) == 1)

        self.publish_phase(first, MaterialHandoffState.HANDOFF_UNLOADED)
        self.publish_phase(first, MaterialHandoffState.HANDOFF_DEPARTED)
        self.completion_publisher.publish(self.make_completion(first))
        first_id = tuple(first.delivery_id.uuid)
        self.spin_until(lambda: self.count_ready(first_id) == 1)
        self.spin_until(lambda: self.count_cycle_goals(first_id) == 1)

        self.completion_publisher.publish(self.make_completion(second))
        duplicate_deadline = time.monotonic() + 0.3
        while time.monotonic() < duplicate_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(self.count_ready(first_id), 1)
        self.assertEqual(self.count_ready(second_id), 1)
        self.assertEqual(self.count_cycle_goals(first_id), 1)
        self.assertEqual(self.count_cycle_goals(second_id), 1)
        self.spin_until(self.all_admitted_cycles_terminal)

    def make_trigger(self, identity, quantity):
        trigger = HandoffTrigger()
        trigger.header.stamp = self.node.get_clock().now().to_msg()
        trigger.delivery_id.uuid[0] = identity
        trigger.material_type = MISSION_TARGET
        trigger.quantity = quantity
        trigger.valid = True
        return trigger

    def publish_phase(self, trigger, phase):
        state = MaterialHandoffState()
        state.header.stamp = self.node.get_clock().now().to_msg()
        state.delivery_id = trigger.delivery_id
        state.phase = phase
        state.valid = True
        self.handoff_publisher.publish(state)
        rclpy.spin_once(self.node, timeout_sec=0.02)

    def make_completion(self, trigger):
        completion = TransferCompletion()
        completion.header.stamp = self.node.get_clock().now().to_msg()
        completion.delivery_id = trigger.delivery_id
        completion.material_type = trigger.material_type
        completion.requested_quantity = trigger.quantity
        completion.supplied_quantity = trigger.quantity
        completion.completed = True
        completion.valid = True
        return completion

    def count_ready(self, delivery_id):
        return sum(
            tuple(state.delivery_id.uuid) == delivery_id
            and state.material_ready
            and state.valid
            for state in self.readiness
        )

    def record_cycle_feedback(self, message):
        delivery_id = tuple(message.feedback.delivery_id.uuid)
        goal_id = tuple(message.goal_id.uuid)
        self.cycle_goal_ids_by_delivery.setdefault(delivery_id, set()).add(goal_id)

    def count_cycle_goals(self, delivery_id):
        return len(self.cycle_goal_ids_by_delivery.get(delivery_id, set()))

    def record_cycle_status(self, message):
        self.cycle_goal_status = {
            tuple(status.goal_info.goal_id.uuid): status.status
            for status in message.status_list
        }

    def all_admitted_cycles_terminal(self):
        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }
        goal_ids = {
            goal_id
            for goal_ids in self.cycle_goal_ids_by_delivery.values()
            for goal_id in goal_ids
        }
        return len(goal_ids) == 2 and all(
            self.cycle_goal_status.get(goal_id) in terminal_statuses
            for goal_id in goal_ids
        )
