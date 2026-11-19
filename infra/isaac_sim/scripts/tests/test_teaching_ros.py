"""FK feedback failures never promote incomplete robot observations to teaching."""
from concurrent.futures import Future
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.teaching_ros import TeachingRosBridge


@pytest.mark.parametrize('names,positions', [(['tool'], [0.]), (['j1'], [0.]),
    (['j1', 'j2'], [0.]), (['j1', 'j1', 'j2'], [0., 0., 0.]),
    (['j1', 'j2'], [0., math.nan])])
def test_only_complete_finite_arm_snapshot_can_feed_fk(names, positions):
    bridge = TeachingRosBridge.__new__(TeachingRosBridge)
    bridge.arm_joint_names = ('j1', 'j2')
    bridge.query = None
    bridge.pose_at = time.monotonic()
    bridge._joints(NS(name=names, position=positions))
    assert bridge.joints is None
    assert bridge.pose_at == float('-inf')
    complete = NS(name=['tool', 'j2', 'j1'], position=[.2, .3, .4])
    bridge._joints(complete)
    assert bridge.joints is complete


def test_failed_fk_future_invalidates_pose_without_breaking_bridge(monkeypatch):
    from sensor_msgs.msg import JointState
    bridge = TeachingRosBridge.__new__(TeachingRosBridge)
    bridge.query = None
    bridge.operation = None
    bridge.motion_instance = ''
    bridge.query_at = 0.
    bridge.joints = JointState(name=['j1', 'j2'], position=[0., 0.])
    bridge.joints_at = bridge.pose_at = 10.
    bridge.pose = [0., 0., 0., 0., 0., 0., 1.]
    bridge.motion = None
    bridge.motion_at = 0.
    bridge.pending = False
    bridge.goal = None
    future = Future()
    bridge.fk = NS(service_is_ready=lambda: True, call_async=lambda _: future)
    bridge.action = NS(server_is_ready=lambda: False)
    packets = []
    bridge.send = packets.append
    monkeypatch.setattr('tape_roll_cell.teaching_ros.time.monotonic', lambda: 10.)
    bridge.publish()
    future.set_exception(RuntimeError('FK service disconnected'))
    bridge.fk.service_is_ready = lambda: False
    bridge.publish()
    assert bridge.query is None
    assert bridge.pose_at == float('-inf')
    assert packets[-1]['pose_valid'] is False


def test_late_action_acceptance_after_cancel_is_immediately_canceled():
    bridge = TeachingRosBridge.__new__(TeachingRosBridge)
    bridge.pending = False
    bridge.goal = None
    bridge.cancel_requested = False
    future = Future()
    bridge.action = NS(server_is_ready=lambda: True, send_goal_async=lambda _: future)
    bridge.send = lambda _: None
    bridge.command(dict(op='teaching_move', request_id='test', motion={}))
    bridge.command(dict(op='teaching_cancel'))
    canceled = []
    result = Future()
    goal = NS(accepted=True, cancel_goal_async=lambda: canceled.append(True), get_result_async=lambda: result)
    future.set_result(goal)
    assert canceled == [True]
    result.set_result(NS(status=5, result=NS(result_code=5, diagnostic_detail='canceled')))
    assert not bridge.pending and bridge.goal is None


def test_lost_goal_acceptance_is_bounded_and_late_acceptance_is_canceled(monkeypatch):
    from sensor_msgs.msg import JointState
    bridge = TeachingRosBridge.__new__(TeachingRosBridge)
    bridge.pending, bridge.goal, bridge.operation = False, None, None
    bridge.cancel_requested = False
    bridge.motion_instance = 'old-motion'
    bridge.motion = None
    bridge.motion_at = 0.
    bridge.query = None
    bridge.joints = None
    bridge.pose = None
    bridge.pose_at = float('-inf')
    clock = [10.]
    monkeypatch.setattr('tape_roll_cell.teaching_ros.time.monotonic', lambda: clock[0])
    acceptance = Future()
    bridge.action = NS(server_is_ready=lambda: True, send_goal_async=lambda _: acceptance)
    packets = []
    bridge.send = packets.append
    bridge.command(dict(op='teaching_move', request_id='lost', motion={}))
    clock[0] = 21.
    bridge.publish()
    failed = next(packet for packet in packets if packet['op'] == 'teaching_result')
    assert failed['uncertain'] and failed['requires_restart']
    assert not bridge.pending
    canceled = []
    acceptance.set_result(NS(accepted=True, cancel_goal_async=lambda: canceled.append(True)))
    assert canceled == [True]


def test_humble_single_argument_callback_tracks_unique_motion_endpoint_across_restart():
    bridge = TeachingRosBridge.__new__(TeachingRosBridge)
    publishers = [NS(endpoint_gid=[1, 2, 3])]
    bridge.node = NS(get_publishers_info_by_topic=lambda topic: publishers)
    bridge._motion(NS())
    assert bridge.motion_instance == '010203'
    publishers[:] = [NS(endpoint_gid=[4, 5, 6])]
    bridge._motion(NS())
    assert bridge.motion_instance == '040506'
    publishers.append(NS(endpoint_gid=[7, 8, 9]))
    bridge._motion(NS())
    assert bridge.motion_instance == ''
    assert bridge.motion_at == float('-inf')


@pytest.mark.parametrize('invalidate', ['incomplete_feedback', 'joint_settings'])
def test_invalidated_feedback_cannot_be_restored_by_late_fk(invalidate):
    import rclpy
    from sensor_msgs.msg import JointState
    from moveit_msgs.srv import GetPositionFK
    from geometry_msgs.msg import PoseStamped
    rclpy.init()
    node = rclpy.create_node('teaching_fk_invalidation_test')
    bridge = TeachingRosBridge(node, lambda _: None)
    try:
        bridge.command(dict(op='teaching_settings', arm_joint_names=['j1', 'j2']))
        future = Future()
        future.set_running_or_notify_cancel()
        bridge.fk = NS(service_is_ready=lambda: True, call_async=lambda _: future)
        bridge._joints(JointState(name=['j1', 'j2'], position=[0., 0.]))
        bridge.publish()
        if invalidate == 'incomplete_feedback':
            bridge._joints(JointState(name=['j1'], position=[0.]))
        else:
            bridge.command(dict(op='teaching_settings', arm_joint_names=['new_j1', 'new_j2']))
        if not future.cancelled():
            response = GetPositionFK.Response()
            response.error_code.val = 1
            pose = PoseStamped()
            pose.pose.orientation.w = 1.
            response.pose_stamped = [pose]
            future.set_result(response)
        packets = []
        bridge.send = packets.append
        bridge.publish()
        assert packets[-1]['pose_valid'] is False
    finally:
        bridge.close()
        node.destroy_node()
        rclpy.shutdown()


def test_motion_state_topic_supplies_idle_and_process_identity():
    import rclpy
    from arm_cell_interfaces.msg import MotionStatus
    rclpy.init()
    node = rclpy.create_node('teaching_motion_state_test')
    packets = []
    bridge = TeachingRosBridge(node, packets.append)
    publisher = node.create_publisher(MotionStatus, '/motion/state', 10)
    state = MotionStatus(execution_state=MotionStatus.MOTION_STATE_IDLE,
                         backend_inactivity_confirmed=True)
    try:
        deadline = time.monotonic()+3.
        while bridge.motion is None and time.monotonic()<deadline:
            publisher.publish(state)
            rclpy.spin_once(node, timeout_sec=.05)
        bridge.publish()
        assert packets[-1]['motion_idle'] is True
        assert packets[-1]['motion_instance']
    finally:
        bridge.close()
        node.destroy_node()
        rclpy.shutdown()
