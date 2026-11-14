import copy
import json
from pathlib import Path

import pytest
import yaml

from arm_cell_bringup.runtime import compile_mission, load_profile, mission_directory, resolve

ROOT = Path(__file__).resolve().parents[4]
SRC = ROOT / 'ros2_ws/src'
PROFILE = SRC / 'bringup/config/operational_profile.yaml'
MISSIONS = SRC / 'bringup/config/missions'


def test_composition_preserves_operational_behavior():
    config = load_profile(PROFILE)
    assert config['motion']['pick_strategy'] == 'PLANNED'
    assert config['motion']['object_to_grasp_tcp_translation'] == [0, 0, 0.01]
    recipe = json.loads((Path(config['orchestration']['recipe_directory']) / 'TestCube.json').read_text())
    pose = recipe['place']['desired_object_pose']
    assert config['motion']['place_waypoint_target_pose_xyzw'] == pose['position'] + pose['orientation_xyzw']
    assert Path(config['vision']['calibration_file']).is_file()


def test_material_offset_does_not_override_global_policy(tmp_path):
    mission = yaml.safe_load((MISSIONS / 'TestCube.yaml').read_text())
    material_path = resolve(mission['material'], MISSIONS)
    material = yaml.safe_load(material_path.read_text())
    material['pick_offset']['translation_m'] = [0.01, 0.02, 0.03]
    material_path = tmp_path / 'material.yaml'
    material_path.write_text(yaml.safe_dump(material))
    mission['material'] = str(material_path)
    mission_path = tmp_path / 'TestCube.yaml'
    mission_path.write_text(yaml.safe_dump(mission))
    recipe, actual_material = compile_mission(mission_path)
    baseline, _ = compile_mission(MISSIONS / 'TestCube.yaml')
    assert recipe['pick'] == baseline['pick']
    assert actual_material['pick_offset']['translation_m'] == [0.01, 0.02, 0.03]


def test_unknown_material_is_rejected(tmp_path):
    mission = yaml.safe_load((MISSIONS / 'TestCube.yaml').read_text())
    mission['target_id'] = 'Other'
    path = tmp_path / 'Other.yaml'
    path.write_text(yaml.safe_dump(mission))
    with pytest.raises(ValueError, match='must match'):
        compile_mission(path)


def test_package_path_cannot_escape():
    with pytest.raises(ValueError):
        resolve('package://arm_cell_vision/../../outside', ROOT)


def test_generated_transport_is_reproducible():
    first = mission_directory(MISSIONS)
    second = mission_directory(MISSIONS)
    assert first == second
    assert json.loads((Path(first) / 'TestCube.json').read_text())['schema_version'] == 1


def test_material_transform_reaches_motion_without_changing_pick_policy(tmp_path):
    mission = yaml.safe_load((MISSIONS / 'TestCube.yaml').read_text())
    material = yaml.safe_load(resolve(mission['material'], MISSIONS).read_text())
    material['pick_offset']['translation_m'] = [0.01, -0.02, 0.03]
    material['pick_offset']['quaternion_xyzw'] = [0, 0, 1, 0]
    material_path = tmp_path / 'material.yaml'
    material_path.write_text(yaml.safe_dump(material))
    mission['material'] = str(material_path)
    mission_path = tmp_path / 'TestCube.yaml'
    mission_path.write_text(yaml.safe_dump(mission))
    orchestration = yaml.safe_load((SRC / 'orchestration/config/operational.yaml').read_text())
    orchestration['recipe_directory'] = str(tmp_path)
    # Keep the mission folder separate from the owner files.
    mission_folder = tmp_path / 'missions'
    mission_folder.mkdir()
    mission_path.rename(mission_folder / mission_path.name)
    orchestration['recipe_directory'] = str(mission_folder)
    owner_path = tmp_path / 'orchestration.yaml'
    owner_path.write_text(yaml.safe_dump(orchestration))
    profile = yaml.safe_load(PROFILE.read_text())
    profile['components']['orchestration'] = str(owner_path)
    for key in ('simulation',):
        profile['components'][key] = str(resolve(profile['components'][key], PROFILE.parent))
    profile_path = tmp_path / 'deployment.yaml'
    profile_path.write_text(yaml.safe_dump(profile))
    actual = load_profile(profile_path)
    baseline = load_profile(PROFILE)
    assert actual['motion']['object_to_grasp_tcp_translation'] == [0.01, -0.02, 0.03]
    assert actual['motion']['object_to_grasp_tcp_quaternion'] == [0, 1, 0, 0]
    actual_recipe = json.loads((Path(actual['orchestration']['recipe_directory']) / 'TestCube.json').read_text())
    baseline_recipe = json.loads((Path(baseline['orchestration']['recipe_directory']) / 'TestCube.json').read_text())
    assert actual_recipe['pick'] == baseline_recipe['pick']


def test_selected_robot_tool_cell_values_reach_runtime(tmp_path):
    profile = yaml.safe_load(PROFILE.read_text())
    for owner, key, value in (
        ('robot', 'retract_joint_positions', [0.1] * 6),
        ('tool', 'finger_contact_span_m', 0.031),
        ('cell', 'place_waypoint_clearance_y_boundary_m', -0.8),
    ):
        owner_path = resolve(profile['profiles'][owner], PROFILE.parent)
        selected = yaml.safe_load(owner_path.read_text())
        section = 'grasp_motion' if owner == 'tool' else 'motion'
        selected[section][key] = value
        path = tmp_path / f'{owner}.yaml'
        path.write_text(yaml.safe_dump(selected))
        profile['profiles'][owner] = str(path)
    for key in ('simulation',):
        profile['components'][key] = str(resolve(profile['components'][key], PROFILE.parent))
    path = tmp_path / 'deployment.yaml'
    path.write_text(yaml.safe_dump(profile))
    actual = load_profile(path)
    assert actual['motion']['retract_joint_positions'] == [0.1] * 6
    assert actual['motion']['finger_contact_span_m'] == 0.031
    assert actual['motion']['place_waypoint_clearance_y_boundary_m'] == -0.8
    assert actual['motion']['planning_planner_id'] == 'RRTConnect'


def test_duplicate_owner_is_rejected_instead_of_silently_overridden(tmp_path):
    profile = yaml.safe_load(PROFILE.read_text())
    owner = yaml.safe_load(resolve(profile['components']['motion'], PROFILE.parent).read_text())
    owner['arm_joint_names'] = ['wrong']
    motion = tmp_path / 'motion.yaml'
    motion.write_text(yaml.safe_dump(owner))
    profile['components']['motion'] = str(motion)
    for key in ('simulation',):
        profile['components'][key] = str(resolve(profile['components'][key], PROFILE.parent))
    path = tmp_path / 'deployment.yaml'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='Duplicate configuration owner.*arm_joint_names'):
        load_profile(path)


def test_cell_location_and_tool_orientation_reach_recipe_and_motion(tmp_path):
    profile = yaml.safe_load(PROFILE.read_text())
    cell = yaml.safe_load(resolve(profile['profiles']['cell'], PROFILE.parent).read_text())
    location = tmp_path / 'location.yaml'
    location.write_text(yaml.safe_dump({'frame': 'base_link', 'position': [0.4, 0.2, 0.3],
                                       'orientation_xyzw': [0., 0., 0., 1.]}))
    cell['work_locations']['cnc_loading'] = str(location)
    cell['calibration_file'] = 'selected_calibration.yaml'
    (tmp_path / 'selected_calibration.yaml').write_text(
        resolve('package://arm_cell_vision/config/calibration/isaac_overhead_wide.yaml', PROFILE.parent).read_text())
    selected_cell = tmp_path / 'cell.yaml'
    selected_cell.write_text(yaml.safe_dump(cell))
    tool = yaml.safe_load(resolve(profile['profiles']['tool'], PROFILE.parent).read_text())
    tool['motion']['tool_axis_quaternion_xyzw'] = [0., 1., 0., 0.]
    tool_path = tmp_path / 'tool.yaml'
    tool_path.write_text(yaml.safe_dump(tool))
    profile['profiles'].update(cell=str(selected_cell), tool=str(tool_path))
    profile['components']['simulation'] = str(resolve(profile['components']['simulation'], PROFILE.parent))
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    actual = load_profile(path)
    recipe = json.loads((Path(actual['orchestration']['recipe_directory']) / 'TestCube.json').read_text())
    assert actual['motion']['place_waypoint_target_pose_xyzw'] == [0.4, 0.2, 0.3, 0., 0., 0., 1.]
    assert recipe['place']['desired_object_pose']['position'] == [0.4, 0.2, 0.3]
    assert recipe['object_to_grasp_tcp']['quaternion_xyzw'] == [0., 1., 0., 0.]
    assert actual['motion']['object_to_grasp_tcp_quaternion'] == [0., 1., 0., 0.]
    assert actual['vision']['calibration_file'] == str(tmp_path / 'selected_calibration.yaml')
    assert recipe['pick'] == compile_mission(MISSIONS / 'TestCube.yaml')[0]['pick']


def test_owner_selections_require_complete_set(tmp_path):
    profile = yaml.safe_load(PROFILE.read_text())
    del profile['profiles']['tool']
    profile['components']['simulation'] = str(resolve(profile['components']['simulation'], PROFILE.parent))
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='exactly robot, tool and cell'):
        load_profile(path)


def test_validation_excludes_production_cell_constraints_and_preserves_tool():
    config = load_profile(PROFILE.parent / 'pnp_validation_profile.yaml')
    assert 'place_waypoint_clearance_y_boundary_m' not in config['motion']
    assert config['motion']['object_to_grasp_tcp_quaternion'] == [1., 0., 0., 0.]
    assert config['motion']['place_tray_return_joint_positions'] == load_profile(PROFILE)['motion']['place_tray_return_joint_positions']
    assert config['capture'] == load_profile(PROFILE)['gripper_capture']


def test_task_execution_cannot_override_robot_owned_joint_poses(tmp_path):
    profile = yaml.safe_load(PROFILE.read_text())
    task = yaml.safe_load(resolve(profile['components']['task_execution'], PROFILE.parent).read_text())
    task['retract_joint_positions'] = [9.] * 6
    task_path = tmp_path / 'task.yaml'
    task_path.write_text(yaml.safe_dump(task))
    profile['components']['task_execution'] = str(task_path)
    profile['components']['simulation'] = str(resolve(profile['components']['simulation'], PROFILE.parent))
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='Duplicate configuration owner.*retract_joint_positions'):
        load_profile(path)


@pytest.mark.parametrize('section,key,value', [
    ('motion', 'validation_grasp_contact_links', ['wrong']),
    ('planning_scene', 'attachment_link', 'wrong'),
])
def test_validation_cannot_override_tool_owned_links(tmp_path, section, key, value):
    profile = yaml.safe_load((PROFILE.parent / 'pnp_validation_profile.yaml').read_text())
    destination = profile['motion'] if section == 'motion' else profile['validation']['planning_scene']
    destination[key] = value
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='Duplicate configuration owner'):
        load_profile(path)


def test_logical_profiles_do_not_select_the_combined_model():
    config = load_profile(PROFILE)
    assert 'model' not in config['robot']
    assert 'compatible_robot_model' not in config['tool']
    assert Path(config['model_configuration']['urdf']).is_file()
    assert config['model_configuration']['robot_id'] == config['robot']['robot_id']
    assert config['model_configuration']['tool_id'] == config['tool']['tool_id']


def test_common_composition_is_independent_of_fixture_marker(tmp_path):
    from arm_cell_bringup.runtime import compose_owner_profiles
    profile = yaml.safe_load(PROFILE.read_text())
    first = copy.deepcopy(profile)
    second = copy.deepcopy(profile)
    second['fixture'] = {'only_a_marker': True}
    compose_owner_profiles(first, PROFILE)
    compose_owner_profiles(second, PROFILE)
    assert first['motion'] == second['motion']
    assert first['gripper_capture'] == second['gripper_capture']


def test_validation_motion_inputs_follow_explicit_profile_selection(tmp_path):
    profile = yaml.safe_load((PROFILE.parent / 'pnp_validation_profile.yaml').read_text())
    profile['motion_profile_sections']['tool'].append('grasp_motion')
    profile['motion_profile_sections']['cell'] = ['motion']
    path = tmp_path / 'validation.yaml'
    path.write_text(yaml.safe_dump(profile))
    actual = load_profile(path)
    assert actual['motion']['finger_contact_span_m'] == 0.02
    assert actual['motion']['place_waypoint_clearance_y_boundary_m'] == -0.926


@pytest.mark.parametrize('owner', ['robot', 'tool'])
def test_combined_model_must_match_selected_logical_profiles(tmp_path, owner):
    profile = yaml.safe_load(PROFILE.read_text())
    selected = yaml.safe_load(resolve(profile['profiles'][owner], PROFILE.parent).read_text())
    selected[f'{owner}_id'] = 'other'
    owner_path = tmp_path / f'{owner}.yaml'
    owner_path.write_text(yaml.safe_dump(selected))
    profile['profiles'][owner] = str(owner_path)
    profile['components']['simulation'] = str(resolve(profile['components']['simulation'], PROFILE.parent))
    path = tmp_path / 'deployment.yaml'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match=f'Selected model is incompatible with {owner} profile'):
        load_profile(path)
