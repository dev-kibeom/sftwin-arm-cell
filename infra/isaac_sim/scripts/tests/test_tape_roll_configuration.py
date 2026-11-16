"""The two finger lengths use one material and one horizontal fixture sequence."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import xacro
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'infra/isaac_sim/scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/bringup'))
from tape_roll_cell.configuration import compose
from internal_grasp_geometry import ToolGeometry, link_transforms


@pytest.mark.parametrize('suffix',['','_long'])
def test_horizontal_roll_is_compatible_with_selected_tool_and_taught_endpoints(suffix):
    config=compose(ROOT/f'ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10{suffix}.yaml',
        ROOT/'infra/isaac_sim/scripts/tape_roll_cell/experiment.yaml')
    model=config['model_configuration']
    urdf=ET.fromstring(xacro.process_file(model['urdf'],mappings={'mesh_root':model['mesh_root']}).toxml())
    geometry=ToolGeometry(urdf,config['tool']['runtime'])
    experiment=config['cell']['tape_roll']
    material=experiment['material']
    solution=geometry.solve_bore(experiment['insertion_depth_m'],material['width_m'],material['inner_radius_m'])
    assert solution['contracted_swept_radius_m']+.001 < material['inner_radius_m']
    for name,center in [('insert','supply_center_m'),('transfer','receiver_center_m')]:
        pose=experiment['teaching'][name]
        transform=link_transforms(urdf,dict(zip(config['robot']['motion']['arm_joint_names'],pose)))['sf_grasp_tcp']
        assert transform[:3,2] == pytest.approx([1,0,0],abs=1e-6)
        roll_center=transform@np.array([0,0,-experiment['insertion_depth_m'],1])
        expected=np.array(experiment[center],dtype=float)
        if name == 'transfer': expected[2] += experiment['release_clearance_m']
        assert roll_center[:3] == pytest.approx(expected,abs=1e-6)


@pytest.mark.parametrize('suffix', ['', '_long'])
def test_each_cartridge_slot_has_kinematically_correct_horizontal_pick_and_receiver(suffix):
    config = compose(ROOT/f'ros2_ws/src/bringup/config/cell_executions/m0609_hand_e_c10{suffix}_tape.yaml')
    model = config['model_configuration']
    urdf = ET.fromstring(xacro.process_file(model['urdf'], mappings={'mesh_root':model['mesh_root']}).toxml())
    layout = config['cell']['tape_roll']
    assert len(layout['slots']) == 3
    assert config['execution_scope'] == 'cell_physical_fixture'
    assert config['cell']['execution_support']['ros_mission'] is False
    for slot in layout['slots']:
        for name, center in [('insert', 'supply_center_m'), ('transfer', 'receiver_center_m')]:
            transform = link_transforms(urdf, dict(zip(config['robot']['motion']['arm_joint_names'],
                                                        slot['teaching'][name])))['sf_grasp_tcp']
            assert transform[:3, 2] == pytest.approx([1, 0, 0], abs=1e-6)
            actual = transform @ np.array([0, 0, -layout['insertion_depth_m'], 1])
            expected = np.array(slot[center])
            if name == 'transfer':
                expected[2] += layout['release_clearance_m']
            assert actual[:3] == pytest.approx(expected, abs=1e-6)
        for name in ('approach', 'extract'):
            transform = link_transforms(urdf, dict(zip(config['robot']['motion']['arm_joint_names'],
                                                        slot['teaching'][name])))['sf_grasp_tcp']
            assert transform[:3, 2] == pytest.approx([1, 0, 0], abs=1e-6)
            assert transform[0, 3] < slot['supply_center_m'][0] - .05


def test_ros_launcher_rejects_physical_fixture_without_synthesizing_vision():
    from arm_cell_bringup.runtime import load_profile
    with pytest.raises(ValueError, match='ROS Mission execution is unsupported'):
        load_profile(ROOT/'ros2_ws/src/bringup/config/cell_executions/m0609_hand_e_c10_long_tape.yaml')


@pytest.mark.parametrize('suffix', ['', '_long'])
def test_horizontal_trajectories_preserve_axis_and_stay_on_cartesian_line_between_ik_knots(suffix):
    config = compose(ROOT/f'ros2_ws/src/bringup/config/cell_executions/m0609_hand_e_c10{suffix}_tape.yaml')
    model = config['model_configuration']
    urdf = ET.fromstring(xacro.process_file(model['urdf'], mappings={'mesh_root':model['mesh_root']}).toxml())
    names = config['robot']['motion']['arm_joint_names']
    for slot in config['cell']['tape_roll']['slots']:
        for name in ('insert', 'extract'):
            knots = np.array(slot['trajectories'][name])
            start = link_transforms(urdf, dict(zip(names, knots[0])))['sf_grasp_tcp']
            end = link_transforms(urdf, dict(zip(names, knots[-1])))['sf_grasp_tcp']
            assert start[1:3, 3] == pytest.approx(end[1:3, 3], abs=1e-6)
            for a, b in zip(knots, knots[1:]):
                for fraction in (0., .5, 1.):
                    actual = link_transforms(urdf, dict(zip(names, a+(b-a)*fraction)))['sf_grasp_tcp']
                    assert actual[1:3, 3] == pytest.approx(start[1:3, 3], abs=5e-5)
                    assert actual[:3, 2] == pytest.approx([1., 0., 0.], abs=5e-5)
