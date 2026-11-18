"""Formal Cell inputs retain physical geometry and horizontal Cartesian paths."""
import sys
import copy
from pathlib import Path
from functools import lru_cache
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import xacro
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'infra/isaac_sim/scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/bringup'))
from arm_cell_bringup.runtime import load_profile
from gripper_runtime.radial_geometry import ToolGeometry,link_transforms


@lru_cache(maxsize=2)
def selected(suffix):
    config=load_profile(ROOT/f'ros2_ws/src/bringup/config/tape_m0609_c10{suffix}_profile.yaml')
    model=config['model_configuration']
    urdf=ET.fromstring(xacro.process_file(model['urdf'],mappings={'mesh_root':model['mesh_root']}).toxml())
    return config,urdf


@pytest.mark.parametrize('suffix',['','_long'])
def test_each_slot_has_compatible_bore_and_correct_horizontal_endpoints(suffix):
    config,urdf=selected(suffix)
    layout=config['cell']['tape_roll'];material=layout['material']
    solution=ToolGeometry(urdf,config['tool']['runtime']).solve_bore(layout['insertion_depth_m'],material['width_m'],material['inner_radius_m'])
    assert solution['contracted_swept_radius_m']+.001 < material['inner_radius_m']
    for slot in layout['slots']:
        for name,center in [('insert','supply_center_m'),('transfer','receiver_center_m')]:
            pose=slot['teaching'][name]
            transform=link_transforms(urdf,dict(zip(config['robot']['motion']['arm_joint_names'],pose)))['sf_grasp_tcp']
            axis=[1,0,0] if name=='insert' else [0,0,-1]
            assert transform[:3,2]==pytest.approx(axis,abs=1e-6)
            actual=transform@np.array([0,0,-layout['insertion_depth_m'],1])
            expected=np.array(slot[center])
            if name=='transfer':expected[2]+=layout['release_clearance_m']
            assert actual[:3]==pytest.approx(expected,abs=1e-6)


@pytest.mark.parametrize('suffix',['','_long'])
def test_cartesian_insert_and_extract_remain_axial_between_ik_knots(suffix):
    config,urdf=selected(suffix);names=config['robot']['motion']['arm_joint_names']
    for slot in config['cell']['tape_roll']['slots']:
        for name in ('insert','extract'):
            knots=np.array(slot['trajectories'][name])
            start=link_transforms(urdf,dict(zip(names,knots[0])))['sf_grasp_tcp']
            for left,right in zip(knots,knots[1:]):
                actual=link_transforms(urdf,dict(zip(names,(left+right)/2)))['sf_grasp_tcp']
                assert actual[1:3,3]==pytest.approx(start[1:3,3],abs=5e-5)
                assert actual[:3,2]==pytest.approx([1,0,0],abs=5e-5)


@pytest.mark.parametrize('suffix',['','_long'])
def test_ros_paths_begin_at_robot_home_and_use_native_configured_tasks(suffix):
    import json
    config,_=selected(suffix)
    registry=json.loads(config['motion']['configured_grasp_cycles_json'])
    motions=json.loads(config['motion']['taught_motions_json'])
    entry=motions[registry['slot_0']['approach_motion']]
    assert entry['plan_to_start'] is True
    assert entry['joint_path'][0]!=config['home_joint_positions']
    assert config['execution_scope']=='operational'
    assert 'vision' not in config


@pytest.mark.parametrize('suffix',['','_long'])
def test_indexed_supply_reuses_fixed_datum_and_connected_return_path(suffix):
    config,_=selected(suffix)
    assert config['cell']['motion_preparation']['ready'],config['cell']['motion_preparation']
    slots=config['cell']['tape_roll']['slots']
    first=slots[0]
    for slot in slots:
        assert slot['supply_center_m']==first['supply_center_m']
        assert slot['receiver_center_m']==first['receiver_center_m']
    paths=slots[1]['trajectories']
    previous=first['teaching']['retreat']
    for segment in ('return_place','return_turn','return_clearance','return_forward','approach'):
        assert paths[segment][0]==pytest.approx(previous,abs=1e-8)
        previous=paths[segment][-1]
    assert previous==pytest.approx(first['teaching']['approach'],abs=1e-8)


@pytest.mark.parametrize('suffix',['','_long'])
def test_tool_turn_keeps_the_actual_mount_position_fixed(suffix):
    config,urdf=selected(suffix)
    paths=config['cell']['tape_roll']['slots'][0]['trajectories']
    names=config['robot']['motion']['arm_joint_names']
    positions=[link_transforms(urdf,dict(zip(names,q)))['tool_mount'][:3,3] for q in paths['turn']]
    assert positions==pytest.approx(np.repeat([positions[0]],len(positions),axis=0),abs=1e-5)


def test_cartesian_timing_extends_to_fit_selected_robot_limits():
    from tape_roll_cell.configuration import _timed_path
    path=[[0.]*6,[.05,0.,0.,0.,0.,0.],[.4,0.,0.,0.,0.,0.]]
    result=_timed_path([path,list(reversed(path))],1.,[1.]*6,[2.]*6)
    samples=np.array(result['joint_path']);dt=result['duration_s']/(len(samples)-1)
    velocity=np.diff(samples,axis=0)/dt
    acceleration=np.diff(np.vstack([np.zeros(6),velocity,np.zeros(6)]),axis=0)/dt
    assert result['duration_s']>=1.
    assert samples[0]==pytest.approx(path[0])
    assert samples[-1]==pytest.approx(path[0])
    assert np.max(samples[:,0])==pytest.approx(path[-1][0])
    assert np.max(abs(velocity))<=.3
    assert np.max(abs(acceleration))<=.4


def test_acceleration_retiming_does_not_scale_duration_as_acceleration():
    from tape_roll_cell.configuration import _timed_path
    result=_timed_path([[[0.],[1.]]],.1,[100.],[5.])
    samples=np.array(result['joint_path']);dt=.02
    velocity=np.diff(samples,axis=0)/dt
    acceleration=np.diff(np.vstack([np.zeros(1),velocity,np.zeros(1)]),axis=0)/dt
    assert np.max(abs(acceleration))<=1.
    # A cubic unit stroke under 1 rad/s² needs about sqrt(6) seconds, not
    # the tens of seconds produced by a linear acceleration-ratio multiplier.
    assert 2.4 < result['duration_s'] < 3.5
