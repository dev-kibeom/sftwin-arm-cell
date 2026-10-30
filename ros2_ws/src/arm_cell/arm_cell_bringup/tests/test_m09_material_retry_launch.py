"""Exercise owner-routed failure followed by a fresh delivery retry."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import rclpy
from arm_cell_interfaces.msg import (
    HandoffTrigger,
    MaterialHandoffState,
    MaterialReadiness,
    TransferCompletion,
)


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
                    package="arm_cell_gateway",
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
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.node = rclpy.create_node(
            "m09_material_retry_test", use_global_arguments=False, context=self.context
        )
        self.executor.add_node(self.node)
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
        self.trigger_publisher = self.node.create_publisher(
            HandoffTrigger, "/external/handoff_trigger", 10
        )
        self.completion_publisher = self.node.create_publisher(
            TransferCompletion, "/material_supply/transfer_completion", 10
        )

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def spin_until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            self.executor.spin_once(timeout_sec=0.02)

    def request(self, identity_byte):
        trigger = HandoffTrigger()
        trigger.header.stamp = self.node.get_clock().now().to_msg()
        trigger.delivery_id.uuid[0] = identity_byte
        trigger.quantity = 1
        trigger.material_type = "RawPart"
        trigger.valid = True
        self.trigger_publisher.publish(trigger)
        return trigger

    def test_failed_delivery_then_fresh_request_reaches_ready(self):
        self.spin_until(
            lambda: self.node.count_subscribers("/external/handoff_trigger") > 0
        )
        self.spin_until(
            lambda: self.node.count_subscribers("/vda/material_handoff_state") >= 2
        )
        self.spin_until(
            lambda: self.node.count_publishers("/integration/material_readiness") >= 1
        )

        failed_trigger = self.request(31)
        failed_id = failed_trigger.delivery_id
        self.spin_until(
            lambda: any(
                item.delivery_id == failed_id and not item.material_ready
                for item in self.readiness
            )
        )
        failure = TransferCompletion()
        failure.header.stamp = self.node.get_clock().now().to_msg()
        failure.delivery_id = failed_id
        failure.material_type = "RawPart"
        failure.requested_quantity = 1
        failure.supplied_quantity = 0
        failure.completed = False
        failure.valid = True
        failure.failure_reason = TransferCompletion.TRANSFER_FAILURE_SUPPLY
        self.completion_publisher.publish(failure)
        self.assertFalse(
            any(
                tuple(state.delivery_id.uuid) == tuple(failed_id.uuid)
                and state.material_ready
                and state.valid
                for state in self.readiness
            )
        )

        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == tuple(failed_id.uuid)
                and not state.material_ready
                for state in self.readiness
            )
        )
        retried_trigger = self.request(32)
        retried_id = retried_trigger.delivery_id
        self.assertNotEqual(tuple(retried_id.uuid), tuple(failed_id.uuid))
        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == tuple(retried_id.uuid)
                and state.phase == MaterialHandoffState.HANDOFF_DEPARTED
                for state in self.handoffs
            )
        )
        completion = TransferCompletion()
        completion.header.stamp = self.node.get_clock().now().to_msg()
        completion.delivery_id = retried_id
        completion.material_type = "RawPart"
        completion.requested_quantity = 1
        completion.supplied_quantity = 1
        completion.completed = True
        completion.valid = True
        self.completion_publisher.publish(completion)
        self.spin_until(
            lambda: any(
                tuple(state.delivery_id.uuid) == tuple(retried_id.uuid)
                and state.material_ready
                and state.valid
                for state in self.readiness
            )
        )
