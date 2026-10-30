"""Owner-routed VDA to Integration material handoff launch check."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import pytest
import rclpy
from arm_cell_interfaces.msg import MaterialHandoffState, MaterialReadiness
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from unique_identifier_msgs.msg import UUID
from arm_cell_interfaces.srv import RequestMaterialSupply


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
                ),
                Node(
                    package="arm_cell_integration",
                    executable="material_handoff_node",
                    name="material_handoff",
                    output="screen",
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestMaterialHandoffLaunch(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "material_handoff_launch_test", use_global_arguments=False
        )
        self.phases = []
        self.readiness = []
        self.late_readiness = []
        self.node.create_subscription(
            MaterialHandoffState,
            "/vda/material_handoff_state",
            self.phases.append,
            10,
        )
        self.node.create_subscription(
            MaterialReadiness,
            "/integration/material_readiness",
            self.readiness.append,
            10,
        )
        self.client = self.node.create_client(
            RequestMaterialSupply, "/integration/request_material"
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_request_preserves_delivery_identity_through_readiness_and_departure(self):
        self.spin_until(self.client.service_is_ready)
        request = RequestMaterialSupply.Request()
        request.request_id = UUID()
        request.request_id.uuid[0] = 7
        future = self.client.call_async(request)
        self.spin_until(future.done)
        response = future.result()
        self.assertTrue(response.accepted, response.diagnostic_detail)

        self.spin_until(
            lambda: any(
                not message.material_ready
                and message.valid
                and tuple(message.delivery_id.uuid) == tuple(response.delivery_id.uuid)
                for message in self.readiness
            )
        )
        self.spin_until(
            lambda: any(
                message.material_ready and message.valid for message in self.readiness
            )
        )
        ready = next(
            message
            for message in self.readiness
            if message.material_ready and message.valid
        )
        self.assertEqual(
            tuple(response.delivery_id.uuid), tuple(ready.delivery_id.uuid)
        )
        late_subscription = self.node.create_subscription(
            MaterialReadiness,
            "/integration/material_readiness",
            self.late_readiness.append,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.spin_until(
            lambda: any(
                message.material_ready
                and message.valid
                and tuple(message.delivery_id.uuid) == tuple(response.delivery_id.uuid)
                for message in self.late_readiness
            )
        )
        phases = [
            message.phase
            for message in self.phases
            if tuple(message.delivery_id.uuid) == tuple(ready.delivery_id.uuid)
        ]
        expected = [
            MaterialHandoffState.HANDOFF_ARRIVED,
            MaterialHandoffState.HANDOFF_DOCKED,
            MaterialHandoffState.HANDOFF_UNLOADED,
            MaterialHandoffState.HANDOFF_DEPARTED,
        ]
        self.spin_until(
            lambda: any(
                tuple(message.delivery_id.uuid) == tuple(ready.delivery_id.uuid)
                and message.phase == MaterialHandoffState.HANDOFF_DEPARTED
                for message in self.phases
            )
        )
        phases = [
            message.phase
            for message in self.phases
            if tuple(message.delivery_id.uuid) == tuple(ready.delivery_id.uuid)
        ]
        cursor = 0
        for phase in phases:
            if cursor < len(expected) and phase == expected[cursor]:
                cursor += 1
        self.assertEqual(cursor, len(expected))
        self.assertTrue(
            all(
                message.valid
                for message in self.phases
                if tuple(message.delivery_id.uuid) == tuple(ready.delivery_id.uuid)
            )
        )
        # Let Integration's subscription process the same terminal VDA fact
        # before issuing the next owner-routed request.
        next_delivery_ready = time.monotonic() + 0.1
        self.spin_until(lambda: time.monotonic() >= next_delivery_ready)

        next_request = RequestMaterialSupply.Request()
        next_request.request_id = UUID()
        next_request.request_id.uuid[0] = 8
        next_future = self.client.call_async(next_request)
        self.spin_until(next_future.done)
        next_response = next_future.result()
        self.assertTrue(next_response.accepted, next_response.diagnostic_detail)
        self.assertNotEqual(
            tuple(next_response.delivery_id.uuid), tuple(response.delivery_id.uuid)
        )
        self.spin_until(
            lambda: any(
                tuple(message.delivery_id.uuid) == tuple(next_response.delivery_id.uuid)
                and message.valid
                and not message.material_ready
                for message in self.late_readiness
            )
        )
        self.assertFalse(
            any(
                message.material_ready
                and tuple(message.delivery_id.uuid) == tuple(response.delivery_id.uuid)
                for message in self.late_readiness[-1:]
            ),
            "the new delivery must replace the previously retained READY state",
        )
        self.spin_until(
            lambda: any(
                message.material_ready
                and message.valid
                and tuple(message.delivery_id.uuid)
                == tuple(next_response.delivery_id.uuid)
                for message in self.readiness
            )
        )
        self.spin_until(
            lambda: any(
                tuple(message.delivery_id.uuid) == tuple(next_response.delivery_id.uuid)
                and message.phase == MaterialHandoffState.HANDOFF_DEPARTED
                for message in self.phases
            )
        )
        self.node.destroy_subscription(late_subscription)
