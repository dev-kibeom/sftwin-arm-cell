"""Verify the production VDA-to-Integration-to-Orchestration path in ROS."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from action_msgs.msg import GoalStatusArray
from arm_cell_interfaces.action import ExecuteCycle
from arm_cell_interfaces.msg import MaterialHandoffState, MaterialReadiness, SafetyState
from arm_cell_interfaces.srv import RequestMaterialSupply
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from unique_identifier_msgs.msg import UUID


MISSION_TARGET = "synthetic-production-flow"


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
                    package="arm_cell_integration",
                    executable="material_handoff_node",
                    name="material_handoff_node",
                    output="screen",
                    parameters=[
                        {
                            "transfer_duration_ms": 100,
                            "source_freshness_timeout_ms": 500,
                        }
                    ],
                ),
                Node(
                    package="arm_cell_bt",
                    executable="orchestration_node",
                    name="orchestration_node",
                    output="screen",
                    parameters=[{"mission_target_id": MISSION_TARGET}],
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestProductionMaterialFlow(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "production_material_flow_test", use_global_arguments=False
        )
        self.handoffs = []
        self.readiness = []
        self.status = []
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
        self.node.create_subscription(
            GoalStatusArray,
            "/orchestration/execute_cycle/_action/status",
            self.status.append,
            10,
        )
        self.node.create_subscription(Log, "/rosout", self.logs.append, 10)
        self.safety_publisher = self.node.create_publisher(
            SafetyState, "/safety/state", 10
        )
        self.request_client = self.node.create_client(
            RequestMaterialSupply, "/integration/request_material"
        )
        self.execute_client = ActionClient(
            self.node, ExecuteCycle, "/orchestration/execute_cycle"
        )
        self.safety_timer = self.node.create_timer(0.05, self.publish_safe_state)

    def tearDown(self):
        self.safety_timer.cancel()
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

    def spin_until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_request_reaches_one_orchestration_admission_with_same_identity(self):
        self.spin_until(self.request_client.service_is_ready)
        self.spin_until(self.execute_client.server_is_ready)
        self.spin_until(
            lambda: self.node.count_subscribers("/vda/material_handoff_state") >= 2
        )
        self.spin_until(
            lambda: self.node.count_subscribers("/integration/material_readiness") >= 2
        )
        self.spin_until(lambda: self.node.count_subscribers("/safety/state") >= 1)

        request = RequestMaterialSupply.Request()
        request.request_id = UUID()
        request.request_id.uuid[0] = 77
        response_future = self.request_client.call_async(request)
        self.spin_until(response_future.done)
        response = response_future.result()
        self.assertTrue(response.accepted, response.diagnostic_detail)
        delivery_id = tuple(response.delivery_id.uuid)

        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == delivery_id
                and state.phase == MaterialHandoffState.HANDOFF_DEPARTED
                and state.valid
                for state in self.handoffs
            )
        )
        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == delivery_id
                and state.material_ready
                and state.valid
                for state in self.readiness
            )
        )

        delivery_hex = "".join(f"{value:02x}" for value in delivery_id)
        self.spin_until(
            lambda: any(len(status.status_list) == 1 for status in self.status)
        )
        self.spin_until(
            lambda: any(
                "mission_failure origin=" in record.msg
                and f"delivery_id={delivery_hex}" in record.msg
                for record in self.logs
            )
        )
