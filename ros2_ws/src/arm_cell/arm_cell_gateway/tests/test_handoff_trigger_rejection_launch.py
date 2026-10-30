"""Gateway rejects stale external triggers without readiness."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import pytest
import rclpy
from arm_cell_interfaces.msg import (
    HandoffTrigger,
    HandoffTriggerStatus,
    MaterialReadiness,
)


@pytest.mark.launch_test
def generate_test_description():
    return (
        launch.LaunchDescription(
            [
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


class TestRejectedMaterialTrigger(unittest.TestCase):
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.node = rclpy.create_node(
            "material_trigger_rejection_test",
            use_global_arguments=False,
            context=self.context,
        )
        self.executor.add_node(self.node)
        self.readiness = []
        self.forwarded = []
        self.status = []
        self.node.create_subscription(
            MaterialReadiness,
            "/integration/material_readiness",
            self.readiness.append,
            10,
        )
        self.node.create_subscription(
            HandoffTrigger, "/integration/accepted_handoff", self.forwarded.append, 10
        )
        self.node.create_subscription(
            HandoffTriggerStatus,
            "/integration/handoff_trigger_status",
            self.status.append,
            10,
        )
        self.publisher = self.node.create_publisher(
            HandoffTrigger, "/external/handoff_trigger", 10
        )

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            self.executor.spin_once(timeout_sec=0.02)

    def test_stale_trigger_is_rejected_without_forwarding_or_ready(self):
        self.spin_until(
            lambda: self.node.count_subscribers("/external/handoff_trigger") > 0
        )
        trigger = HandoffTrigger()
        trigger.header.stamp.sec = 1
        trigger.delivery_id.uuid[0] = 9
        trigger.quantity = 2
        trigger.material_type = "RawPart"
        trigger.valid = True
        self.publisher.publish(trigger)
        self.spin_until(lambda: len(self.status) == 1)
        self.assertEqual(self.status[0].state, HandoffTriggerStatus.TRIGGER_REJECTED)
        self.assertEqual(
            self.status[0].rejection_reason,
            HandoffTriggerStatus.REJECTION_STALE_SOURCE_TIMESTAMP,
        )
        self.assertEqual(self.forwarded, [])
        self.assertFalse(any(item.material_ready for item in self.readiness))
