"""Frame teaching is geometry, independent of robot IK and simulator state."""
import copy
import math
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.teaching_program import (
    validate_program, resolve_point, resolve_program, capture_point, pose_matrix,
    pose7_matrix, default_program, apply_layout, equipment_transform)


def program():
    return dict(schema_version=1,frames=dict(infeed=dict(pose=[1.,2.,3.,0.,0.,math.pi/2]),outfeed=dict(pose=[0.]*6)),
        points=dict(pick=dict(frame='infeed',pose=[.1,0.,0.,0.,0.,0.],mode='linear',velocity_scale=.1,acceleration_scale=.1)),
        sequences={name:['pick'] for name in ('tape_approach','tape_insert','tape_transfer','tape_retreat')})


def test_rotation_and_robot_base_transform_are_composed_in_se3():
    p=program();base=dict(x=.3,y=-.2,z=.1,roll=.1,pitch=.2,yaw=-.4)
    actual=pose7_matrix(resolve_point(p,'pick',base))
    expected=np.linalg.inv(pose_matrix([.3,-.2,.1,.1,.2,-.4])) @ pose_matrix(p['frames']['infeed']['pose']) @ pose_matrix(p['points']['pick']['pose'])
    assert actual==pytest.approx(expected)
    assert resolve_point(p,'pick',{})[:3]==pytest.approx([1.,2.1,3.])


def test_capture_round_trip_preserves_selected_frame_and_input():
    p=program();before=copy.deepcopy(p)
    pose=[.2,.3,.4,0.,0.,1.,0.];base=dict(x=1.,yaw=.5)
    captured=capture_point(p,'pick',pose,base)
    assert captured['points']['pick']['frame']=='infeed'
    assert pose7_matrix(resolve_point(captured,'pick',base))==pytest.approx(pose7_matrix(pose))
    assert p==before


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),True,'0.1'])
def test_nonfinite_or_untyped_units_are_rejected(bad):
    p=program();p['points']['pick']['pose'][0]=bad
    with pytest.raises(ValueError):validate_program(p)
    p=program();p['points']['pick']['velocity_scale']=bad
    with pytest.raises(ValueError):validate_program(p)


def test_sequences_are_explicit_sparse_named_points():
    p=program();p['points']['other']=dict(p['points']['pick'],frame='base',pose=[0.]*6)
    p['sequences']['tape_insert']=['other','pick']
    motions=resolve_program(p,{})
    assert len(motions['tape_insert']['cartesian_waypoints'])==2
    assert motions['tape_insert']['cartesian_waypoints'][0]['motion']=='MoveL'
    del p['points']['other']
    with pytest.raises(ValueError):validate_program(p)


def test_independent_outfeed_se3_moves_fixture_and_receipt_but_not_pick_or_floor():
    import yaml
    root=Path(__file__).resolve().parents[4]
    cell=yaml.safe_load((root/'ros2_ws/src/bringup/config/cells/tape_roll_cell.yaml').read_text())
    cell['tape_roll']['material']=yaml.safe_load((root/'ros2_ws/src/bringup/config/materials/tape_roll/17mm.yaml').read_text())
    config=dict(cell=cell);cell['tape_roll']['slots']=[dict(index=0)]
    p=default_program(config);apply_layout(config,p)
    before=copy.deepcopy(cell['tape_roll']);pick=resolve_point(p,'insert',{})
    p['frames']['outfeed']['pose']=[.2,-.5,.6,.1,.2,.8]
    apply_layout(config,p)
    after=cell['tape_roll']
    assert resolve_point(p,'insert',{})==pytest.approx(pick)
    assert after['slots'][0]['supply_center_m']==before['slots'][0]['supply_center_m']
    assert after['slots'][0]['receiver_center_m']!=before['slots'][0]['receiver_center_m']
    fixed=lambda layout:[box for box in layout['static_boxes'] if box['equipment'] is None]
    assert fixed(before)==fixed(after)
    assert equipment_transform(config,'outfeed')[:3,:3]!=pytest.approx(np.eye(3))


def test_motion_names_select_bt_compositions_independently_of_point_geometry():
    p = program()
    for name in ('MoveJ', 'MoveL', 'MoveJChecked', 'MoveLChecked', 'MovePlanned'):
        p['points']['pick']['mode'] = name
        waypoint = resolve_program(p, {})['tape_insert']['cartesian_waypoints'][0]
        assert waypoint['motion'] == name
        assert 'mode' not in waypoint
        assert waypoint['pose'] == pytest.approx(resolve_point(p, 'pick', {}))


def test_blend_is_opt_in_at_the_junction_with_the_next_point():
    p = program()
    p['points']['extract'] = dict(p['points']['pick'], blend=True)
    p['sequences']['tape_transfer'] = ['extract', 'pick']
    waypoints = resolve_program(p, {})['tape_transfer']['cartesian_waypoints']
    assert waypoints[0].get('blend') is True
    assert not waypoints[1].get('blend', False)


def test_blend_cannot_cross_the_end_of_a_motion():
    p = program()
    p['points']['pick']['blend'] = True
    with pytest.raises(ValueError, match='blend'):
        resolve_program(p, {})


@pytest.mark.parametrize('model_id, blends_into_place', [
    ('m0609_hand_e_c10_long', True), ('m0609_hand_e_c10', False),
])
def test_default_program_blends_transfer_and_speeds_up_empty_tool(model_id, blends_into_place):
    cell = dict(tape_roll=dict(
        supply_center_m=[.53, .35, .415], insertion_depth_m=.014,
        extract_lift_m=.1, release_clearance_m=.004,
        material=dict(width_m=.017, outer_radius_m=.075),
        place=dict(first_center_xy_m=[.3, -.35], surface_z_m=.3,
                   turn_position_m=[.3, -.35, .38], transport_height_m=.72,
                   approach_clearance_m=.08)))
    config = dict(cell=cell, model_configuration=dict(model_id=model_id))
    motions = resolve_program(default_program(config), {})
    transfer = motions['tape_transfer']
    assert [point.get('blend', False) for point in transfer['cartesian_waypoints']] == [
        True, True, True, True, blends_into_place, False]
    for name, motion in motions.items():
        expected = .1 if name == 'tape_transfer' else .15
        assert motion['velocity_scale'] == expected
        assert motion['acceleration_scale'] == expected
