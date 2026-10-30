"""Clock startup and rollback require their own fresh adapter launch fixture."""

import time
import unittest
import uuid
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_ros.actions import Node
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
import xacro


@pytest.mark.launch_test
def generate_test_description():
    model = (
        Path(get_package_share_directory("arm_cell_description"))
        / "urdf/m0609_robotiq_2f85.urdf.xacro"
    )
    # Do not share a latched process or /clock with another scenario. In particular,
    # a publisher in the developer's current ROS domain must not supply startup time.
    prefix = "/arm_cell_clock_test_" + uuid.uuid4().hex
    topics = {name: prefix + "/" + name for name in ("raw", "canonical", "clock")}
    adapter = Node(
        package="arm_cell_sim_adapter",
        executable="joint_state_adapter",
        parameters=[
            {
                "robot_description": xacro.process_file(str(model)).toxml(),
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("raw_joint_states", topics["raw"]),
            ("joint_states", topics["canonical"]),
            ("/clock", topics["clock"]),
        ],
    )
    return launch.LaunchDescription(
        [
            adapter,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"adapter": adapter, "topics": topics}


class TestClock(unittest.TestCase):
    def test_clock_required_and_backward_clock_latches(
        self, adapter, topics, proc_output
    ):
        rclpy.init(args=[])
        node = rclpy.create_node("clock_adapter_test", use_global_arguments=False)
        received = []
        node.create_subscription(JointState, topics["canonical"], received.append, 10)
        raw = node.create_publisher(JointState, topics["raw"], qos_profile_sensor_data)
        clock = node.create_publisher(Clock, topics["clock"], 10)
        msg = JointState()
        msg.header.stamp.sec = 10
        msg.name = [f"joint_{i}" for i in range(1, 7)] + [
            "gripper_robotiq_85_left_knuckle_joint"
        ]
        msg.position = [0.0] * 7

        def send(clock_sec, duration=0.5):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                if clock_sec is not None:
                    tick = Clock()
                    tick.clock.sec = clock_sec
                    clock.publish(tick)
                raw.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.02)

        try:
            end = time.monotonic() + 5.0
            while not (
                raw.get_subscription_count()
                and clock.get_subscription_count()
                and node.count_publishers(topics["canonical"])
            ):
                assert time.monotonic() < end, "adapter endpoints were not discovered"
                rclpy.spin_once(node, timeout_sec=0.02)
            send(None, 1.0)
            assert not received, "clock is a required startup input"
            send(10, 1.0)
            assert received
            send(9)
            proc_output.assertWaitFor(
                "simulation clock moved backward", process=adapter, timeout=5
            )
            received.clear()
            send(11)
            assert (
                not received
            ), "clock rollback requires restart even with monotonic state"
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestAdapterShutdown(unittest.TestCase):
    def test_adapter_exits_cleanly(self, proc_info, adapter):
        launch_testing.asserts.assertExitCodes(proc_info, process=adapter)
