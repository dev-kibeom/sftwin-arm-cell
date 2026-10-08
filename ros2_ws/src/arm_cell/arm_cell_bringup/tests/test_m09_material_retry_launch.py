"""Exercise owner-routed failure followed by a fresh delivery retry."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from arm_cell_interfaces.msg import MaterialHandoffState, MaterialReadiness
from arm_cell_interfaces.srv import RequestMaterialSupply
from std_srvs.srv import SetBool
from unique_identifier_msgs.msg import UUID


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
                    name="material_handoff_node",
                    output="screen",
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestM09MaterialRetry(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "m09_material_retry_test", use_global_arguments=False
        )
        self.handoffs = []
        self.readiness = []
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
        self.request_client = self.node.create_client(
            RequestMaterialSupply, "/integration/request_material"
        )
        self.fault_client = self.node.create_client(
            SetBool, "/external_state_mock/fault/material_handoff_failure"
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def request(self, identity_byte):
        request = RequestMaterialSupply.Request()
        request.request_id = UUID()
        request.request_id.uuid[0] = identity_byte
        future = self.request_client.call_async(request)
        self.spin_until(future.done)
        response = future.result()
        self.assertTrue(response.accepted, response.diagnostic_detail)
        return response.delivery_id

    def set_fault(self, active):
        request = SetBool.Request()
        request.data = active
        future = self.fault_client.call_async(request)
        self.spin_until(future.done)
        self.assertTrue(future.result().success)

    def test_failed_delivery_then_fresh_request_reaches_ready(self):
        self.spin_until(self.request_client.service_is_ready)
        self.spin_until(self.fault_client.service_is_ready)
        self.spin_until(
            lambda: self.node.count_subscribers("/vda/material_handoff_state") >= 2
        )
        self.spin_until(
            lambda: self.node.count_publishers("/integration/material_readiness") >= 1
        )

        failed_id = self.request(31)
        self.set_fault(True)
        self.set_fault(False)
        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == tuple(failed_id.uuid)
                and state.phase == MaterialHandoffState.HANDOFF_FAILED
                for state in self.handoffs
            )
        )
        self.assertFalse(
            any(
                tuple(state.delivery_id.uuid) == tuple(failed_id.uuid)
                and state.material_ready
                and state.valid
                for state in self.readiness
            )
        )

        # The negative readiness is published only after Integration consumes
        # the VDA-owned failed phase and closes the active delivery.
        self.assertEqual(
            self.handoffs[-1].phase,
            MaterialHandoffState.HANDOFF_FAILED,
        )
        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == tuple(failed_id.uuid)
                and not state.material_ready
                for state in self.readiness
            )
        )
        retried_id = self.request(32)
        self.assertNotEqual(tuple(retried_id.uuid), tuple(failed_id.uuid))
        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == tuple(retried_id.uuid)
                and state.material_ready
                and state.valid
                for state in self.readiness
            )
        )
