"""Cell startup validates bore geometry and exports sparse frame teaching."""
import sys
import json
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
from gripper_runtime.radial_geometry import ToolGeometry
from tape_roll_cell.teaching_program import resolve_point,pose7_matrix,base_matrix


@lru_cache(maxsize=2)
def selected(suffix):
    config=load_profile(ROOT/f'ros2_ws/src/bringup/config/tape_m0609_c10{suffix}_profile.yaml')
    model=config['model_configuration']
    urdf=ET.fromstring(xacro.process_file(model['urdf'],mappings={'mesh_root':model['mesh_root']}).toxml())
    return config,urdf


@pytest.mark.parametrize('suffix',['','_long'])
def test_geometry_derived_pick_place_centres_and_tcp_axes(suffix):
    config,urdf=selected(suffix);cell=config['cell'];layout=cell['tape_roll'];material=layout['material']
    solution=ToolGeometry(urdf,config['tool']['runtime']).solve_bore(layout['insertion_depth_m'],material['width_m'],material['inner_radius_m'])
    assert solution['contracted_swept_radius_m']+.001 < material['inner_radius_m']
    for name,center,axis in [('insert','supply_center_m',[1,0,0]),('place','receiver_center_m',[0,0,-1])]:
        transform=pose7_matrix(resolve_point(cell['teaching_program'],name,cell['world_to_base']))
        assert transform[:3,2]==pytest.approx(axis,abs=1e-8)
        actual=transform @ [0.,0.,-layout['insertion_depth_m'],1.]
        expected=np.array(layout['slots'][0][center])
        if name=='place':expected[2]+=layout['release_clearance_m']
        assert actual[:3]==pytest.approx(expected,abs=1e-8)


@pytest.mark.parametrize('suffix',['','_long'])
def test_all_indexed_slots_reference_same_sparse_motion(suffix):
    config,_=selected(suffix);layout=config['cell']['tape_roll']
    registry=json.loads(config['motion']['configured_grasp_cycles_json'])
    motions=json.loads(config['motion']['taught_motions_json'])
    assert config['cell']['motion_preparation']['ready']
    assert set(motions)=={'home','tape_approach','tape_insert','tape_transfer','tape_retreat'}
    for operation in registry.values():
        assert operation['approach_motion']=='tape_approach'
        assert operation['transfer_motion']=='tape_transfer'
    assert sum(len(m.get('cartesian_waypoints',[])) for m in motions.values())<20
    assert all('joint_path' not in m for m in motions.values())
    assert all('teaching' not in slot and 'trajectories' not in slot for slot in layout['slots'])
    assert all(slot['supply_center_m']==layout['slots'][0]['supply_center_m'] for slot in layout['slots'])
    assert config['execution_scope']=='operational' and 'vision' not in config


def test_startup_does_not_import_numerical_ik_solver():
    source=(ROOT/'infra/isaac_sim/scripts/tape_roll_cell/configuration.py').read_text()
    assert 'TeachingSolver' not in source and 'least_squares' not in source
    yaml=(ROOT/'ros2_ws/src/bringup/config/cells/tape_roll_cell.yaml').read_text()
    assert '  teaching:' not in yaml


@pytest.mark.parametrize('suffix', ['', '_long'])
def test_default_turn_uses_the_selected_tools_configured_turn_position(suffix):
    config, _ = selected(suffix)
    cell = config['cell']
    turn = resolve_point(cell['teaching_program'], 'turn', cell['world_to_base'])
    world = base_matrix(cell['world_to_base']) @ pose7_matrix(turn)
    assert world[:3, 3] == pytest.approx(cell['tape_roll']['place']['turn_position_m'])
