"""Gateway accepts external triggers and gates readiness on owner evidence."""

import copy
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from arm_cell_interfaces.msg import (
    HandoffTrigger,
    HandoffTriggerStatus,
    MaterialHandoffState,
    MaterialReadiness,
    TransferCompletion,
)


@pytest.mark.launch_test
def generate_test_description():
    material_handoff = Node(
        package="arm_cell_gateway",
        executable="material_handoff_node",
        name="material_handoff_node",
    )
    return (
        launch.LaunchDescription(
            [
                material_handoff,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"material_handoff": material_handoff},
    )


class TestMaterialHandoffLaunch(unittest.TestCase):
    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.node = rclpy.create_node(
            "material_handoff_launch_test",
            use_global_arguments=False,
            context=self.context,
        )
        self.executor.add_node(self.node)
        self.accepted = []
        self.trigger_status = []
        self.readiness = []
        self.node.create_subscription(
            HandoffTrigger, "/integration/accepted_handoff", self.accepted.append, 10
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
            QoSProfile(
                depth=64,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.trigger_publisher = self.node.create_publisher(
            HandoffTrigger, "/external/handoff_trigger", 10
        )
        self.vda_publisher = self.node.create_publisher(
            MaterialHandoffState, "/vda/material_handoff_state", 10
        )
        self.completion_publisher = self.node.create_publisher(
            TransferCompletion, "/material_supply/transfer_completion", 10
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

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def test_trigger_is_forwarded_unchanged_and_requires_full_supply_and_vda(
        self, material_handoff, proc_output
    ):
        self.spin_for(0.1)
        self.spin_until(
            lambda: self.node.count_subscribers("/external/handoff_trigger") > 0
        )
        trigger = HandoffTrigger()
        trigger.header.stamp = self.node.get_clock().now().to_msg()
        trigger.delivery_id.uuid[0] = 42
        trigger.quantity = 3
        trigger.material_type = "RawPart"
        trigger.has_random_seed = True
        trigger.random_seed = 1234
        trigger.valid = True
        self.trigger_publisher.publish(trigger)
        self.spin_until(lambda: len(self.accepted) == 1)
        self.spin_until(lambda: len(self.trigger_status) == 1)
        self.assertEqual(self.trigger_status[0].delivery_id, trigger.delivery_id)
        self.assertEqual(
            self.trigger_status[0].state, HandoffTriggerStatus.TRIGGER_ACCEPTED
        )
        accepted = self.accepted[0]
        self.assertEqual(accepted.delivery_id, trigger.delivery_id)
        self.assertEqual(accepted.quantity, trigger.quantity)
        self.assertEqual(accepted.material_type, trigger.material_type)
        self.assertTrue(accepted.has_random_seed)
        self.assertEqual(accepted.random_seed, trigger.random_seed)
        self.spin_until(lambda: len(self.readiness) >= 1)
        self.assertFalse(self.readiness[-1].material_ready)

        for phase in (
            MaterialHandoffState.HANDOFF_ARRIVED,
            MaterialHandoffState.HANDOFF_DOCKED,
            MaterialHandoffState.HANDOFF_UNLOADED,
            MaterialHandoffState.HANDOFF_DEPARTED,
        ):
            state = MaterialHandoffState()
            state.header.stamp = self.node.get_clock().now().to_msg()
            state.delivery_id = trigger.delivery_id
            state.phase = phase
            state.valid = True
            self.vda_publisher.publish(state)
            self.executor.spin_once(timeout_sec=0.03)

        self.spin_for(0.6)
        self.assertFalse(
            any(
                item.delivery_id == trigger.delivery_id and item.material_ready
                for item in self.readiness
            )
        )
        completion = self.make_completion(trigger)
        self.completion_publisher.publish(completion)
        self.spin_until(
            lambda: any(
                item.delivery_id == trigger.delivery_id and item.material_ready
                for item in self.readiness
            )
        )
        proc_output.assertWaitFor(
            "material handoff supply completed requested=3 supplied=3",
            process=material_handoff,
        )
        ready = next(
            item
            for item in self.readiness
            if item.delivery_id == trigger.delivery_id and item.material_ready
        )
        self.assertTrue(ready.valid)
        self.assertEqual(ready.delivery_id, trigger.delivery_id)
        self.assertEqual(ready.material_type, trigger.material_type)
        self.assertEqual(ready.quantity, trigger.quantity)

        trigger.header.stamp = self.node.get_clock().now().to_msg()
        self.trigger_publisher.publish(trigger)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        self.assertEqual(len(self.accepted), 1)
        self.spin_until(lambda: len(self.trigger_status) == 2)
        self.assertEqual(
            self.trigger_status[-1].state, HandoffTriggerStatus.TRIGGER_REJECTED
        )
        self.assertEqual(
            self.trigger_status[-1].rejection_reason,
            HandoffTriggerStatus.REJECTION_DUPLICATE_DELIVERY_ID,
        )
        second = HandoffTrigger()
        second.header.stamp = self.node.get_clock().now().to_msg()
        second.delivery_id.uuid[0] = 43
        second.quantity = 1
        second.material_type = "RawPart"
        second.valid = True
        self.trigger_publisher.publish(second)
        self.spin_until(lambda: len(self.accepted) == 2)

        mismatched_completion = self.make_completion(second)
        mismatched_completion.delivery_id = copy.deepcopy(second.delivery_id)
        mismatched_completion.delivery_id.uuid[0] = 99
        self.completion_publisher.publish(mismatched_completion)
        self.spin_for(0.1)
        self.assertFalse(
            any(
                item.delivery_id == second.delivery_id and item.material_ready
                for item in self.readiness
            )
        )

        stale_completion = self.make_completion(second)
        stale_completion.header.stamp.sec = 1
        stale_completion.header.stamp.nanosec = 0
        self.completion_publisher.publish(stale_completion)
        self.spin_until(
            lambda: any(
                item.delivery_id == second.delivery_id and not item.valid
                for item in self.readiness
            )
        )
        self.assertFalse(
            any(
                item.delivery_id == second.delivery_id and item.material_ready
                for item in self.readiness
            )
        )
        self.completion_publisher.publish(self.make_completion(second))

        third = HandoffTrigger()
        third.header.stamp = self.node.get_clock().now().to_msg()
        third.delivery_id.uuid[0] = 44
        third.quantity = 1
        third.material_type = "RawPart"
        third.valid = True
        self.trigger_publisher.publish(third)
        self.spin_until(lambda: len(self.accepted) == 3)

        stale_vda = MaterialHandoffState()
        stale_vda.header.stamp.sec = 1
        stale_vda.delivery_id = third.delivery_id
        stale_vda.phase = MaterialHandoffState.HANDOFF_ARRIVED
        stale_vda.valid = True
        self.vda_publisher.publish(stale_vda)
        self.spin_until(
            lambda: any(
                item.delivery_id == third.delivery_id and not item.valid
                for item in self.readiness
            )
        )
        self.assertFalse(
            any(
                item.delivery_id == third.delivery_id and item.material_ready
                for item in self.readiness
            )
        )

        fourth = HandoffTrigger()
        fourth.header.stamp = self.node.get_clock().now().to_msg()
        fourth.delivery_id.uuid[0] = 46
        fourth.quantity = 1
        fourth.material_type = "RawPart"
        fourth.valid = True
        self.trigger_publisher.publish(fourth)
        self.spin_until(lambda: len(self.accepted) == 4)
        invalid_completion = self.make_completion(fourth)
        invalid_completion.valid = False
        self.completion_publisher.publish(invalid_completion)
        self.spin_until(
            lambda: any(
                item.delivery_id == fourth.delivery_id and not item.valid
                for item in self.readiness
            )
        )
        self.assertFalse(
            any(
                item.delivery_id == fourth.delivery_id and item.material_ready
                for item in self.readiness
            )
        )

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

    def test_concurrent_deliveries_keep_vda_completion_and_readiness_correlated(
        self, material_handoff
    ):
        self.spin_until(
            lambda: self.node.count_subscribers("/external/handoff_trigger") > 0
        )
        first = self.make_trigger(71, quantity=2)
        second = self.make_trigger(72, quantity=3)
        self.trigger_publisher.publish(first)
        self.spin_until(lambda: len(self.accepted) == 1)
        self.trigger_publisher.publish(second)
        self.spin_until(lambda: len(self.accepted) == 2)
        self.spin_until(lambda: len(self.trigger_status) == 2)

        for trigger in (first, second):
            for phase in (
                MaterialHandoffState.HANDOFF_ARRIVED,
                MaterialHandoffState.HANDOFF_DOCKED,
                MaterialHandoffState.HANDOFF_UNLOADED,
            ):
                state = MaterialHandoffState()
                state.header.stamp = self.node.get_clock().now().to_msg()
                state.delivery_id = trigger.delivery_id
                state.phase = phase
                state.valid = True
                self.vda_publisher.publish(state)
                self.spin_for(0.04)

        self.completion_publisher.publish(self.make_completion(second))
        self.completion_publisher.publish(self.make_completion(first))
        for trigger in (first, second):
            self.spin_until(
                lambda trigger=trigger: any(
                    item.delivery_id == trigger.delivery_id and item.material_ready
                    for item in self.readiness
                )
            )
        for trigger in (first, second):
            ready = [
                item
                for item in self.readiness
                if item.delivery_id == trigger.delivery_id and item.material_ready
            ]
            self.assertEqual(len(ready), 1)
            self.assertEqual(ready[0].quantity, trigger.quantity)

    def make_trigger(self, identity, *, quantity):
        trigger = HandoffTrigger()
        trigger.header.stamp = self.node.get_clock().now().to_msg()
        trigger.delivery_id.uuid[0] = identity
        trigger.quantity = quantity
        trigger.material_type = "RawPart"
        trigger.valid = True
        return trigger

    def test_supply_failure_reason_and_counts_are_logged(
        self, material_handoff, proc_output
    ):
        self.spin_until(
            lambda: self.node.count_subscribers("/external/handoff_trigger") > 0
        )
        trigger = HandoffTrigger()
        trigger.header.stamp = self.node.get_clock().now().to_msg()
        trigger.delivery_id.uuid[0] = 45
        trigger.quantity = 2
        trigger.material_type = "RawPart"
        trigger.valid = True
        self.trigger_publisher.publish(trigger)
        self.spin_until(lambda: len(self.accepted) == 1)

        completion = self.make_completion(trigger)
        completion.supplied_quantity = 1
        completion.completed = False
        completion.failure_reason = TransferCompletion.TRANSFER_FAILURE_SUPPLY
        self.completion_publisher.publish(completion)
        proc_output.assertWaitFor(
            "material handoff delivery failed reason=supply_failure requested=2 supplied=1",
            process=material_handoff,
        )
        self.assertFalse(
            any(
                item.delivery_id == trigger.delivery_id and item.material_ready
                for item in self.readiness
            )
        )
