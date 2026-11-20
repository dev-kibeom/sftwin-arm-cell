"""Pendant edits remain drafts until acknowledged by the Motion owner."""
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.teaching_program import resolve_point, pose7_matrix, MOTIONS
from tape_roll_cell.teaching_runtime import TeachingSession


@pytest.fixture
def pendant(tmp_path):
    point = dict(frame='infeed', pose=[.1, 0., 0., 0., 0., 0.], mode='linear',
                 velocity_scale=.8, acceleration_scale=.7)
    program = dict(schema_version=1,
                   frames=dict(infeed=dict(pose=[1., 2., 3., 0., 0., math.pi/2]),
                               outfeed=dict(pose=[-1., 0., 0., 0., 0., 0.])),
                   points=dict(approach=point), sequences={name: ['approach'] for name in MOTIONS})
    packets = []
    clock = [10.]
    adapter = NS(configuration=dict(cell=dict(teaching_program=program,
                                             teaching_file=str(tmp_path/'teaching.json'),
                                             world_to_base=dict(x=0., y=0., z=0.)),
                                    motion=dict(taught_motions_json=json.dumps(dict(home=dict(target='home', velocity_scale=.1, acceleration_scale=.1))))),
                 _pending_cycle=None, supply=NS(reservation_id=''),
                 conveyor=NS(active_roll_paths=[]), timeline=NS(is_stopped=lambda: False),
                 stage=NS(GetPrimAtPath=lambda _: False),
                 sidecar=NS(available=True, send=packets.append))
    owner = TeachingSession(adapter, clock=lambda: clock[0])
    owner._test_packets = packets
    owner._test_clock = clock
    owner.consume(dict(op='teaching_state', motion_idle=True, motion_ready=True,
                       pose_valid=True, current_pose=[1., 2.1, 3., 0., 0., 0., 1.]))
    yield owner
    owner.close()


def test_capture_and_frame_selection_preserve_measured_tcp(pendant):
    assert pendant.command('capture', name='approach')['accepted']
    np.testing.assert_allclose(resolve_point(pendant.draft, 'approach', pendant.world_to_base), pendant.pose, atol=1e-12)
    target = pose7_matrix(resolve_point(pendant.draft, 'approach', pendant.world_to_base))
    assert pendant.command('point_frame', name='approach', frame='outfeed')['accepted']
    np.testing.assert_allclose(pose7_matrix(resolve_point(pendant.draft, 'approach', pendant.world_to_base)), target, atol=1e-12)
    assert pendant.active != pendant.draft


def test_atomic_save_reload_and_invalid_edit_leave_valid_program(pendant):
    assert pendant.command('save')['accepted']
    saved = pendant.path.read_bytes()
    bad = deepcopy(pendant.draft['points']['approach'])
    bad['pose'][0] = float('nan')
    assert not pendant.command('edit_point', name='approach', point=bad)['accepted']
    assert pendant.command('save')['accepted']
    assert pendant.path.read_bytes() == saved
    point = deepcopy(pendant.draft['points']['approach'])
    point['pose'][0] += .02
    assert pendant.command('edit_point', name='approach', point=point)['accepted']
    assert pendant.status()['dirty']
    assert pendant.command('load')['accepted']
    assert not pendant.status()['dirty']


def test_manual_movement_uses_capped_scales_and_stale_feedback_is_rejected(pendant):
    assert not pendant.command('move', name='approach')['accepted']
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert pendant.command('move', name='approach')['accepted']
    packet = pendant._test_packets[-1]
    assert packet['motion']['velocity_scale'] == .1
    assert packet['motion']['acceleration_scale'] == .1
    assert not pendant.command('manual_mode', enabled=False)['accepted']
    pendant.consume(dict(op='teaching_result', request_id=packet['request_id'], success=False, detail='canceled'))
    pendant._test_clock[0] += 1.
    assert not pendant.command('capture', name='approach')['accepted']
    assert not pendant.command('jog', frame='base', axis=0, step=.001)['accepted']


def test_jog_follows_selected_equipment_axis_and_limits_step(pendant):
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert not pendant.command('jog', frame='base', axis=0, step=1.)['accepted']
    assert pendant.command('jog', frame='infeed', axis=0, step=.001)['accepted']
    target = pendant._test_packets[-1]['motion']['cartesian_waypoints'][0]['pose']
    np.testing.assert_allclose(target[:3], [1., 2.101, 3.], atol=1e-12)


def test_apply_commits_acknowledged_snapshot_not_later_edits(pendant, monkeypatch):
    import tape_roll_cell.teaching_runtime as runtime
    layouts = []
    monkeypatch.setattr(runtime, 'apply_layout', lambda _, program: layouts.append(deepcopy(program)))
    monkeypatch.setattr(runtime, 'author_equipment_frames', lambda *args: None)
    assert pendant.command('manual_mode', enabled=True)['accepted']
    candidate = deepcopy(pendant.draft)
    assert pendant.command('apply')['accepted']
    request = pendant._test_packets[-1]
    point = deepcopy(pendant.draft['points']['approach'])
    point['pose'][0] += .2
    assert pendant.command('edit_point', name='approach', point=point)['accepted']
    pendant.consume(dict(op='teaching_result', request_id='other', success=True))
    assert pendant.pending is not None
    pendant.consume(dict(op='teaching_result', request_id=request['request_id'], success=True))
    assert pendant.active == candidate
    assert pendant.active != pendant.draft
    assert layouts == [candidate]


def test_equipment_apply_requires_stopped_empty_scene_and_mission_blocks_manual(pendant):
    pendant.adapter._pending_cycle = 'running'
    assert not pendant.command('manual_mode', enabled=True)['accepted']
    pendant.adapter._pending_cycle = None
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert pendant.command('edit_frame', name='infeed', pose=[1., 3., 3., 0., 0., 0.])['accepted']
    assert not pendant.command('apply')['accepted']
    pendant.adapter.timeline.is_stopped = lambda: True
    pendant.adapter.conveyor.active_roll_paths = ['roll']
    assert not pendant.command('apply')['accepted']
    pendant.adapter.conveyor.active_roll_paths = []
    assert pendant.command('apply')['accepted']


def test_partial_scene_apply_failure_blocks_automatic_mode_until_reapplied(pendant, monkeypatch):
    import tape_roll_cell.teaching_runtime as runtime
    monkeypatch.setattr(runtime, 'apply_layout', lambda *_: None)
    def fail(*_):
        raise RuntimeError('Scene unavailable')
    monkeypatch.setattr(runtime, 'author_equipment_frames', fail)
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert pendant.command('apply')['accepted']
    request = pendant._test_packets[-1]
    pendant.consume(dict(op='teaching_result', request_id=request['request_id'], success=True))
    assert pendant.apply_uncertain
    assert not pendant.command('manual_mode', enabled=False)['accepted']
    monkeypatch.setattr(runtime, 'author_equipment_frames', lambda *_: None)
    assert pendant.command('apply')['accepted']
    pendant.consume(dict(op='teaching_result', request_id=pendant._test_packets[-1]['request_id'], success=True))
    assert pendant.command('manual_mode', enabled=False)['accepted']


def test_play_during_pending_layout_apply_never_moves_equipment(pendant, monkeypatch):
    import tape_roll_cell.teaching_runtime as runtime
    writes = []
    monkeypatch.setattr(runtime, 'apply_layout', lambda *_: writes.append('layout'))
    monkeypatch.setattr(runtime, 'author_equipment_frames', lambda *_: writes.append('stage'))
    pendant.adapter.timeline.is_stopped = lambda: True
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert pendant.command('edit_frame', name='infeed', pose=[1., 3., 3., 0., 0., 0.])['accepted']
    assert pendant.command('apply')['accepted']
    packet = pendant._test_packets[-1]
    pendant.adapter.timeline.is_stopped = lambda: False
    pendant.consume(dict(op='teaching_result', request_id=packet['request_id'], success=True))
    assert not writes
    assert pendant.apply_uncertain
    assert not pendant.command('manual_mode', enabled=False)['accepted']


def test_lost_apply_acknowledgement_becomes_uncertain_and_allows_reapplication(pendant):
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert pendant.command('apply')['accepted']
    pendant._test_clock[0] += 16.
    pendant.tick()
    assert pendant.pending is None
    assert pendant.apply_uncertain
    assert not pendant.command('manual_mode', enabled=False)['accepted']
    pendant.consume(dict(op='teaching_state', motion_idle=True, motion_ready=True))
    assert pendant.command('apply')['accepted']


def test_unknown_move_acceptance_requires_new_motion_instance_and_idle_reapply(pendant, monkeypatch):
    import tape_roll_cell.teaching_runtime as runtime
    monkeypatch.setattr(runtime, 'apply_layout', lambda *_: None)
    monkeypatch.setattr(runtime, 'author_equipment_frames', lambda *_: None)
    pendant.consume(dict(op='teaching_state', motion_idle=True, motion_ready=True, motion_instance='old'))
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert pendant.command('move', name='approach')['accepted']
    pendant.consume(dict(op='teaching_result', request_id=pendant.pending['id'],
                         success=False, uncertain=True, requires_restart=True))
    assert pendant.pending is None and pendant.motion_uncertain
    assert not pendant.command('apply')['accepted']
    assert not pendant.command('manual_mode', enabled=False)['accepted']
    pendant.consume(dict(op='teaching_state', motion_idle=True, motion_ready=True, motion_instance='new'))
    assert pendant.command('apply')['accepted']
    pendant.consume(dict(op='teaching_result', request_id=pendant.pending['id'], success=True))
    assert not pendant.motion_uncertain
    assert pendant.command('manual_mode', enabled=False)['accepted']


@pytest.mark.parametrize('operation,values', [
    ('move', dict(name='approach')),
    ('jog', dict(frame='base', axis=0, step=.001)),
])
def test_uncertain_apply_blocks_manual_moves_until_successful_reapply(pendant, monkeypatch, operation, values):
    import tape_roll_cell.teaching_runtime as runtime
    monkeypatch.setattr(runtime, 'apply_layout', lambda *_: None)
    monkeypatch.setattr(runtime, 'author_equipment_frames', lambda *_: None)
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert pendant.command('apply')['accepted']
    pendant.consume(dict(op='teaching_result', request_id=pendant.pending['id'],
                         success=False, uncertain=True))
    sent = len(pendant._test_packets)
    assert not pendant.command(operation, **values)['accepted']
    assert len(pendant._test_packets) == sent
    assert pendant.command('apply')['accepted']
    pendant.consume(dict(op='teaching_result', request_id=pendant.pending['id'], success=True))
    assert pendant.command(operation, **values)['accepted']


def test_captured_movej_reuses_joint_target_after_save_and_load(pendant):
    point = deepcopy(pendant.draft['points']['approach'])
    point['mode'] = 'MoveJ'
    assert pendant.command('edit_point', name='approach', point=point)['accepted']
    joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    pendant.consume(dict(op='teaching_state', motion_idle=True, motion_ready=True,
                        pose_valid=True, current_pose=[1., 2.1, 3., 0., 0., 0., 1.],
                        joint_positions=joints))
    assert pendant.command('capture', name='approach')['accepted']
    assert pendant.command('point_frame', name='approach', frame='outfeed')['accepted']
    assert pendant.command('save')['accepted']
    assert pendant.command('revert')['accepted']
    assert pendant.command('load')['accepted']
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert pendant.command('move', name='approach')['accepted']
    waypoint = pendant._test_packets[-1]['motion']['cartesian_waypoints'][0]
    assert waypoint.get('joint_positions') == joints


def test_movej_capture_requires_joint_feedback_from_pose_sample(pendant):
    point = deepcopy(pendant.draft['points']['approach'])
    point['mode'] = 'MoveJ'
    assert pendant.command('edit_point', name='approach', point=point)['accepted']
    assert not pendant.command('capture', name='approach')['accepted']



def test_moved_frame_does_not_replay_joint_target_at_old_location(pendant):
    point = deepcopy(pendant.draft['points']['approach'])
    point['mode'] = 'MoveJ'
    assert pendant.command('edit_point', name='approach', point=point)['accepted']
    pendant.consume(dict(op='teaching_state', motion_idle=True, motion_ready=True,
                        pose_valid=True, current_pose=[1., 2.1, 3., 0., 0., 0., 1.],
                        joint_positions=[0.1] * 6))
    assert pendant.command('capture', name='approach')['accepted']
    assert pendant.command('edit_frame', name='infeed', pose=[2., 2., 3., 0., 0., math.pi/2])['accepted']
    assert pendant.command('manual_mode', enabled=True)['accepted']
    assert pendant.command('move', name='approach')['accepted']
    waypoint = pendant._test_packets[-1]['motion']['cartesian_waypoints'][0]
    assert waypoint['pose'][:3] == pytest.approx([2., 2.1, 3.])
    assert 'joint_positions' not in waypoint
