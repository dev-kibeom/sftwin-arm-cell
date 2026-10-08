"""Rejected VDA material requests never become Hub acceptance or readiness."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import pytest
import rclpy
from arm_cell_interfaces.msg import MaterialReadiness
from arm_cell_interfaces.srv import RequestMaterial, RequestMaterialSupply
from unique_identifier_msgs.msg import UUID


@pytest.mark.launch_test
def generate_test_description():
    return (
        launch.LaunchDescription(
            [
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


class TestRejectedMaterialRequest(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "material_request_rejection_test", use_global_arguments=False
        )
        self.readiness = []
        self.forwarded = []
        self.node.create_subscription(
            MaterialReadiness,
            "/integration/material_readiness",
            self.readiness.append,
            10,
        )

        def reject(request, response):
            self.forwarded.append(request.delivery_id)
            response.accepted = False
            response.diagnostic_detail = "VDA rejected the request"
            return response

        self.node.create_service(RequestMaterial, "/vda/request_material", reject)
        self.client = self.node.create_client(
            RequestMaterialSupply, "/integration/request_material"
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def test_vda_rejection_returns_false_without_delivery_readiness(self):
        self.spin_until(self.client.service_is_ready)
        request = RequestMaterialSupply.Request()
        request.request_id = UUID()
        request.request_id.uuid[0] = 9
        future = self.client.call_async(request)
        self.spin_until(future.done)

        response = future.result()
        self.assertFalse(response.accepted)
        self.spin_until(lambda: len(self.forwarded) == 1)
        delivery_id = self.forwarded[0]
        self.assertTrue(any(delivery_id.uuid))
        self.assertEqual(tuple(response.delivery_id.uuid), (0,) * 16)

        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertFalse(self.readiness)
