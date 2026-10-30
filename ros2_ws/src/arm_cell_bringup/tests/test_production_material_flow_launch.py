"""Verify the production VDA-to-Integration-to-Orchestration path in ROS."""

import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from arm_cell_interfaces.msg import (
    HandoffTrigger,
    MaterialHandoffState,
    MaterialReadiness,
    SafetyState,
    TransferCompletion,
)
from rcl_interfaces.msg import Log


MISSION_TARGET = "RawPart"


def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                Node(
                    package="arm_cell_vda",
                    executable="external_state_mock_node",
                    name="external_state_mock",
                    output="screen",
                    parameters=[{"material_phase_period_ms": 100}],
                ),
                Node(
                    package="arm_cell_gateway",
                    executable="material_handoff_node",
                    name="material_handoff_node",
                    output="screen",
                    parameters=[
                        {
                            "source_freshness_timeout_ms": 500,
                        }
                    ],
                ),
                Node(
                    package="arm_cell_orchestration",
                    executable="orchestration_node",
                    name="orchestration_node",
                    output="screen",
                    parameters=[
                        {
                            "mission_target_id": MISSION_TARGET,
                            "recipe_directory": str(
                                Path(__file__).resolve().parents[1]
                                / "config"
                                / "recipes"
                            ),
                        }
                    ],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestProductionMaterialFlow(unittest.TestCase):
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.node = rclpy.create_node(
            "production_material_flow_test",
            use_global_arguments=False,
            context=self.context,
        )
        self.executor.add_node(self.node)
        self.handoffs = []
        self.readiness = []
        self.logs = []
        self.node.create_subscription(
            MaterialHandoffState,
            "/vda/material_handoff_state",
            self.handoffs.append,
            10,
        )
        self.node.create_subscription(
            MaterialReadiness,
            "/integration/material_readiness",
            self.readiness.append,
            10,
        )
        self.node.create_subscription(Log, "/rosout", self.logs.append, 10)
        self.safety_publisher = self.node.create_publisher(
            SafetyState, "/safety/state", 10
        )
        self.trigger_publisher = self.node.create_publisher(
            HandoffTrigger, "/external/handoff_trigger", 10
        )
        self.completion_publisher = self.node.create_publisher(
            TransferCompletion, "/material_supply/transfer_completion", 10
        )
        self.safety_timer = self.node.create_timer(0.05, self.publish_safe_state)

    def tearDown(self):
        self.safety_timer.cancel()
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

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

    def spin_until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            self.executor.spin_once(timeout_sec=0.02)

    def test_trigger_readiness_reaches_orchestration_with_same_identity(self):
        self.spin_until(
            lambda: self.node.count_subscribers("/vda/material_handoff_state") >= 2
        )
        self.spin_until(
            lambda: self.node.count_subscribers("/integration/material_readiness") >= 2
        )
        self.spin_until(lambda: self.node.count_subscribers("/safety/state") >= 1)

        trigger = HandoffTrigger()
        trigger.header.stamp = self.node.get_clock().now().to_msg()
        trigger.delivery_id.uuid[0] = 77
        trigger.quantity = 2
        trigger.material_type = "RawPart"
        trigger.has_random_seed = True
        trigger.random_seed = 123
        trigger.valid = True
        delivery_id = tuple(trigger.delivery_id.uuid)
        self.trigger_publisher.publish(trigger)

        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == delivery_id
                and state.phase == MaterialHandoffState.HANDOFF_DEPARTED
                and state.valid
                for state in self.handoffs
            )
        )
        completion = TransferCompletion()
        completion.header.stamp = self.node.get_clock().now().to_msg()
        completion.delivery_id = trigger.delivery_id
        completion.material_type = trigger.material_type
        completion.requested_quantity = trigger.quantity
        completion.supplied_quantity = trigger.quantity
        completion.completed = True
        completion.valid = True
        self.completion_publisher.publish(completion)
        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == delivery_id
                and state.material_ready
                and state.valid
                for state in self.readiness
            )
        )

        self.spin_until(
            lambda: any(
                "material admission wait reason changed "
                "reason=vision_calibration_status_unavailable" in record.msg
                for record in self.logs
            )
        )
