"""Configured supply is a production work source, distinct from Vision acquisitions."""
from pathlib import Path
import json
import pytest
from arm_cell_bringup.runtime import load_profile, export_runtime_configuration
ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize('suffix', ['', '_long'])
def test_tape_profile_selects_real_radial_tool_and_ros_work_source(suffix, tmp_path):
    profile = ROOT/f'ros2_ws/src/bringup/config/tape_m0609_c10{suffix}_profile.yaml'
    config = load_profile(profile)
    assert config['tool']['runtime']['mode_id'] == 'physical_real'
    assert config['tool']['runtime']['supported_combinations'] == [['physics', 'physical']]
    assert 'width' not in config['tool']['runtime']['capabilities']
    assert config['motion']['tool_grasp_strategies'] == ['internal_radial']
    assert config['motion']['grip_force'] == {'actuation': 'none'}
    assert config['orchestration']['material_source'] == 'configured_slots'
    assert 'vision' not in config
    operations = json.loads(config['motion']['configured_grasp_cycles_json'])
    assert set(operations) == {f'slot_{i}' for i in range(config['cell']['tape_roll']['cartridge']['slot_count'])}
    assert all(op['material_id'] == 'TapeRoll17' for op in operations.values())
    result = json.loads(export_runtime_configuration(profile, tmp_path/'runtime.json').read_text())
    assert result['configuration']['_mission_recipe']['source']['type'] == 'configured_slots'


def test_formal_c10_rejects_unsupported_snap_modes():
    with pytest.raises(ValueError, match='combination'):
        load_profile(ROOT/'ros2_ws/src/bringup/config/tape_m0609_c10_profile.yaml',
                     tool_actuator='logical', tool_grasp='fixed_attach')


def test_operational_task_description_keeps_collision_meshes_loadable():
    from xml.dom.minidom import parseString
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters
    from launch_ros.actions import Node
    from arm_cell_bringup.configured_supply_launch import compose
    profile = ROOT/'ros2_ws/src/bringup/config/tape_m0609_c10_long_profile.yaml'
    actions = compose(load_profile(profile), profile)
    task = next(node for node in actions if isinstance(node, Node) and node.node_package == 'arm_cell_task_execution')
    context = LaunchContext()
    (parameters,) = evaluate_parameters(context,task._Node__parameters)
    description = parameters['robot_description']
    meshes = parseString(description).getElementsByTagName('mesh')
    assert meshes
    assert all(mesh.getAttribute('filename').startswith(('file://','package://')) for mesh in meshes)
