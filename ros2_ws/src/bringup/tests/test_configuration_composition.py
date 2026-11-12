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
    assert config['motion']['object_to_grasp_tcp_translation'] == [0, 0, 0]
    recipe = json.loads((Path(config['orchestration']['recipe_directory']) / 'RawPart.json').read_text())
    pose = recipe['place']['desired_object_pose']
    assert config['motion']['place_waypoint_target_pose_xyzw'] == pose['position'] + pose['orientation_xyzw']
    assert Path(config['vision']['calibration_file']).is_file()


def test_material_offset_does_not_override_global_policy(tmp_path):
    mission = yaml.safe_load((MISSIONS / 'RawPart.yaml').read_text())
    material_path = resolve(mission['material'], MISSIONS)
    material = yaml.safe_load(material_path.read_text())
    material['pick_offset']['translation_m'] = [0.01, 0.02, 0.03]
    material_path = tmp_path / 'material.yaml'
    material_path.write_text(yaml.safe_dump(material))
    mission['material'] = str(material_path)
    mission_path = tmp_path / 'RawPart.yaml'
    mission_path.write_text(yaml.safe_dump(mission))
    recipe, actual_material = compile_mission(mission_path)
    baseline, _ = compile_mission(MISSIONS / 'RawPart.yaml')
    assert recipe['pick'] == baseline['pick']
    assert actual_material['pick_offset']['translation_m'] == [0.01, 0.02, 0.03]


def test_unknown_material_is_rejected(tmp_path):
    mission = yaml.safe_load((MISSIONS / 'RawPart.yaml').read_text())
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
    assert json.loads((Path(first) / 'RawPart.json').read_text())['schema_version'] == 1


def test_material_transform_reaches_motion_without_changing_pick_policy(tmp_path):
    mission = yaml.safe_load((MISSIONS / 'RawPart.yaml').read_text())
    material = yaml.safe_load(resolve(mission['material'], MISSIONS).read_text())
    material['pick_offset']['translation_m'] = [0.01, -0.02, 0.03]
    material['pick_offset']['quaternion_xyzw'] = [0, 0, 1, 0]
    material_path = tmp_path / 'material.yaml'
    material_path.write_text(yaml.safe_dump(material))
    mission['material'] = str(material_path)
    mission_path = tmp_path / 'RawPart.yaml'
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
    for key in ('simulation', 'gripper_capture'):
        profile['components'][key] = str(resolve(profile['components'][key], PROFILE.parent))
    profile_path = tmp_path / 'deployment.yaml'
    profile_path.write_text(yaml.safe_dump(profile))
    actual = load_profile(profile_path)
    baseline = load_profile(PROFILE)
    assert actual['motion']['object_to_grasp_tcp_translation'] == [0.01, -0.02, 0.03]
    assert actual['motion']['object_to_grasp_tcp_quaternion'] == [0, 1, 0, 0]
    actual_recipe = json.loads((Path(actual['orchestration']['recipe_directory']) / 'RawPart.json').read_text())
    baseline_recipe = json.loads((Path(baseline['orchestration']['recipe_directory']) / 'RawPart.json').read_text())
    assert actual_recipe['pick'] == baseline_recipe['pick']
