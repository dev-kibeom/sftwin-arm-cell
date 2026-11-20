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
    assert list(parameters['taught_cycle_motion_names']) == [
        'tape_approach', 'tape_insert', 'tape_transfer', 'tape_retreat']
    description = parameters['robot_description']
    meshes = parseString(description).getElementsByTagName('mesh')
    assert meshes
    assert all(mesh.getAttribute('filename').startswith(('file://','package://')) for mesh in meshes)


def test_configured_launch_preserves_common_observation_settings(tmp_path, monkeypatch):
    import importlib.util
    from launch import LaunchContext
    from launch.actions import SetEnvironmentVariable, RegisterEventHandler
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    path=ROOT/'ros2_ws/src/bringup/launch/arm_cell_operational.launch.py'
    spec=importlib.util.spec_from_file_location('tape_operational_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    (tmp_path/'ros2_ws/src').mkdir(parents=True)
    (tmp_path/'infra/isaac_sim/scripts/pnp_cnc_cell').mkdir(parents=True)
    monkeypatch.syspath_prepend(str(ROOT/'infra/isaac_sim/scripts'))
    monkeypatch.setenv('SFTWIN_PROJECT_ROOT',str(tmp_path))
    context=LaunchContext()
    context.launch_configurations.update(profile=str(ROOT/'ros2_ws/src/bringup/config/tape_m0609_c10_long_profile.yaml'),
        include_simulation='true', log_level='error', action_logger_level='warn',
        planning_scene_log_level='warn', diagnostics='motion,safety,orchestration')
    actions=module.compose(context)
    nodes=[action for action in actions if isinstance(action,Node)]
    for node in nodes:
        assert node._ExecuteLocal__output_format==module.CONSOLE_OUTPUT_FORMAT
        assert node._Node__ros_arguments[:2]==['--log-level','error']
        if node.node_package=='arm_cell_vda':continue
        parameters=evaluate_parameters(context,node._Node__parameters)[0]
        assert parameters['diagnostics.capture_enabled'] is True
        assert str(tmp_path) in parameters['diagnostics.output_directory']
    assert any(isinstance(action,RegisterEventHandler) for action in actions)
    environment={context.perform_substitution(action.name[0]):action for action in actions
                 if isinstance(action,SetEnvironmentVariable)}
    assert 'ARM_CELL_RECORDING_SESSION_PATH' in environment
    assert 'RCUTILS_CONSOLE_OUTPUT_FORMAT' in environment
