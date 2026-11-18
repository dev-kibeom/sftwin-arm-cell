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
    assert motions['tape_insert']['cartesian_waypoints'][0]['mode']=='linear'
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


def test_rotated_infeed_surface_commands_follow_its_local_axis():
    from tape_roll_cell.conveyor import Conveyor
    from types import SimpleNamespace as NS
    c=Conveyor.__new__(Conveyor)
    c.configuration={'cell':{'teaching_program':program(),'tape_roll':{'equipment_reference_frames':{
        'infeed':dict(pose=[0.]*6),'outfeed':dict(pose=[0.]*6)}}}}
    measured={}
    def surface(name):
        return NS(GetSurfaceVelocityAttr=lambda:NS(Set=lambda value:measured.update({name:value})))
    c.surfaces=[('/World/TapeRollCell/Infeed/InfeedBelt',surface('infeed')),
                ('/World/TapeRollCell/Outfeed/OutfeedLeft',surface('outfeed'))]
    c.Gf=NS(Vec3f=lambda *values:list(values))
    c._surface(infeed=.04,outfeed=.06)
    assert measured['infeed']==pytest.approx([0.,.04,0.],abs=1e-10)
    assert measured['outfeed']==pytest.approx([0.,-.06,0.])
