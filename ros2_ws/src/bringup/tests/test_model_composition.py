"""Composition rejects inconsistent owner/model inputs before runtime export."""
import copy
import json
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from arm_cell_bringup.runtime import load_profile, resolve

ROOT = Path(__file__).resolve().parents[4]
PROFILE = ROOT / 'ros2_ws/src/bringup/config/operational_profile.yaml'


@pytest.fixture
def selection(tmp_path):
    deployment = yaml.safe_load(PROFILE.read_text())
    documents = {}
    for owner, reference in deployment['profiles'].items():
        source = resolve(reference, PROFILE.parent)
        documents[owner] = yaml.safe_load(source.read_text())
        # Preserve each owner's relative reference base when making test inputs.
        for key in ('home', 'joint_limits', 'geometry', 'static_scene', 'calibration_file', 'capture'):
            if key in documents[owner]:
                documents[owner][key] = str(resolve(documents[owner][key], source.parent))
        deployment['profiles'][owner] = str(tmp_path / f'{owner}.yaml')
    model_path = resolve(deployment['model_configuration'], PROFILE.parent)
    documents['model'] = yaml.safe_load(model_path.read_text())
    for key in ('urdf', 'semantic'):
        documents['model'][key] = str(resolve(documents['model'][key], model_path.parent))
    deployment['model_configuration'] = str(tmp_path / 'model.yaml')
    deployment['components']['simulation'] = str(resolve(deployment['components']['simulation'], PROFILE.parent))

    def compose(change=None):
        selected = copy.deepcopy(documents)
        if change:
            change(selected)
        for name, data in selected.items():
            (tmp_path / f'{name}.yaml').write_text(yaml.safe_dump(data))
        path = tmp_path / 'deployment.yaml'
        path.write_text(yaml.safe_dump(deployment))
        return load_profile(path)

    return compose


def test_owner_metadata_is_exported_without_becoming_motion_parameters():
    config = load_profile(PROFILE)
    assert config['model_configuration']['model_id']
    assert config['model_configuration']['planning_group']
    assert set(config['robot']['joint_policy']['wrap_equivalent_joints']) <= set(config['motion']['arm_joint_names'])
    assert set(config['robot']['joint_policy']['place_ik_seed_joints']) <= set(config['motion']['arm_joint_names'])
    assert config['tool']['contact_links']['payload_touch_links']
    assert config['cell']['contact_objects']['pick_retract']
    assert config['model_configuration']['isaac']['articulation_root']
    assert config['cell']['isaac']['robot_prim_path'].startswith('/')
    assert 'joint_policy' not in config['motion']
    assert 'planning_group' not in config['motion']


@pytest.mark.parametrize('change,message', [
    (lambda d: d['robot']['motion']['arm_joint_names'].reverse(), 'planning group joints'),
    (lambda d: d['robot']['motion']['arm_joint_names'].append('joint_missing'), 'planning group joints'),
    (lambda d: d['robot']['motion']['arm_joint_names'].__setitem__(1, 'joint_1'), 'unique'),
    (lambda d: d['robot']['joint_policy']['wrap_equivalent_joints'].append('unknown'), 'wrap_equivalent_joints'),
    (lambda d: d['robot']['joint_policy']['place_ik_seed_joints'].append('unknown'), 'place_ik_seed_joints'),
    (lambda d: d['robot']['motion']['retract_joint_positions'].__setitem__(0, float('nan')), 'retract_joint_positions'),
    (lambda d: d['robot']['motion']['place_tray_return_joint_positions'].__setitem__(0, 100.), 'place_tray_return_joint_positions'),
    (lambda d: d['model'].__setitem__('model_id', 'wrong_model'), 'model identity'),
    (lambda d: d['model'].__setitem__('planning_group', 'missing_group'), 'planning group'),
    (lambda d: d['tool']['contact_links']['payload_touch_links'].append('missing_link'), 'payload_touch_links'),
    (lambda d: d['tool']['validation_motion']['validation_grasp_contact_links'].append('missing_link'), 'validation_grasp_contact_links'),
    (lambda d: d['tool']['validation_planning_scene'].__setitem__('attachment_link', 'missing_link'), 'attachment_link'),
    (lambda d: d['cell']['contact_objects'].__setitem__('pick_retract', 'missing_object'), 'pick_retract'),
    (lambda d: d['cell']['isaac'].__setitem__('robot_prim_path', 'relative/robot'), 'robot_prim_path'),
    (lambda d: d['model']['isaac'].__setitem__('articulation_root', '../root_joint'), 'articulation_root'),
    (lambda d: d['cell']['work_locations'].__setitem__('cnc_loading', 'missing_location.yaml'), 'missing_location'),
])
def test_invalid_owner_model_combination_is_rejected(selection, change, message):
    with pytest.raises((ValueError, FileNotFoundError), match=message):
        selection(change)


@pytest.mark.parametrize('positions', [[0.], [100.] * 6, [False] * 6, [float('inf')] * 6])
def test_home_must_match_selected_joint_limits(selection, tmp_path, positions):
    home = tmp_path / 'home.yaml'
    home.write_text(yaml.safe_dump({'home_joint_positions': positions}))
    with pytest.raises(ValueError, match='HOME'):
        selection(lambda d: d['robot'].__setitem__('home', str(home)))


@pytest.mark.parametrize('limits', [
    {'unknown': {'has_velocity_limits': True, 'max_velocity': 1.}},
    {'joint_1': {'has_velocity_limits': True, 'max_velocity': -1.}},
    {'joint_1': {'has_velocity_limits': True, 'max_velocity': 1000.}},
])
def test_invalid_robot_planning_limits_are_rejected(selection, tmp_path, limits):
    path = tmp_path / 'limits.yaml'
    path.write_text(yaml.safe_dump({'joint_limits': limits}))
    with pytest.raises(ValueError, match='joint_limits'):
        selection(lambda d: d['robot'].__setitem__('joint_limits', str(path)))


def test_source_export_cli_preserves_owner_metadata(tmp_path):
    path = tmp_path / 'runtime.json'
    result = subprocess.run([
        sys.executable, str(ROOT / 'ros2_ws/src/bringup/scripts/export_runtime_configuration'),
        '--profile', str(PROFILE), '--output', str(path),
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    document = json.loads(path.read_text())
    loaded = load_profile(path)
    for owner in ('robot', 'tool', 'cell', 'model_configuration'):
        assert loaded[owner] == document['configuration'][owner]


def test_selected_home_respects_tighter_planning_limits(selection, tmp_path):
    path = tmp_path / 'limits.yaml'
    path.write_text(yaml.safe_dump({'joint_limits': {'joint_3': {
        'has_position_limits': True, 'min_position': -0.5, 'max_position': 0.5,
    }}}))
    with pytest.raises(ValueError, match='HOME.*joint_limits'):
        selection(lambda d: d['robot'].__setitem__('joint_limits', str(path)))


def test_binding_and_joint_policy_are_owner_data_not_m0609_whitelists(selection):
    def change(d):
        d['robot']['joint_policy']['wrap_equivalent_joints'] = []
        d['robot']['joint_policy']['place_ik_seed_joints'] = ['joint_2']
        d['cell']['isaac']['robot_prim_path'] = '/World/AnotherCell/Robot'
    config = selection(change)
    assert config['robot']['joint_policy']['wrap_equivalent_joints'] == []
    assert config['robot']['joint_policy']['place_ik_seed_joints'] == ['joint_2']
    assert config['cell']['isaac']['robot_prim_path'] == '/World/AnotherCell/Robot'


def test_tcp_must_exist_in_selected_assembly(selection, tmp_path):
    import xacro
    model = yaml.safe_load(resolve(yaml.safe_load(PROFILE.read_text())['model_configuration'], PROFILE.parent).read_text())
    source = resolve(model['urdf'], PROFILE.parent)
    expanded = xacro.process_file(str(source)).toxml()
    path = tmp_path / 'without_tcp.urdf'
    path.write_text(expanded.replace('sf_grasp_tcp', 'different_tcp'))
    with pytest.raises(ValueError, match='sf_grasp_tcp'):
        selection(lambda d: d['model'].__setitem__('urdf', str(path)))


def test_exported_snapshot_does_not_revalidate_source_resources(tmp_path):
    from arm_cell_bringup.runtime import export_runtime_configuration
    path = export_runtime_configuration(PROFILE, tmp_path / 'runtime.json')
    # A snapshot consumer must not re-open source models or add a runtime gate.
    import hashlib
    document = json.loads(path.read_text())
    config = document['configuration']
    # Previously exported schema-v1 snapshots have none of the new metadata.
    for owner, keys in (
        ('robot', ('joint_policy',)),
        ('tool', ('contact_links',)),
        ('cell', ('contact_objects', 'isaac')),
        ('model_configuration', ('model_id', 'planning_group', 'isaac')),
    ):
        for key in keys:
            config[owner].pop(key)
    config['model_configuration']['urdf'] = '/unavailable/source.xacro'
    document['configuration_sha256'] = hashlib.sha256(json.dumps(
        document['configuration'], sort_keys=True, allow_nan=False).encode()).hexdigest()
    path.write_text(json.dumps(document))
    assert load_profile(path)['model_configuration']['urdf'] == '/unavailable/source.xacro'


def test_planning_group_cannot_hide_extra_movable_members(selection, tmp_path):
    import xml.etree.ElementTree as ET
    reference = yaml.safe_load(PROFILE.read_text())['model_configuration']
    model = yaml.safe_load(resolve(reference, PROFILE.parent).read_text())
    srdf = ET.parse(resolve(model['semantic'], PROFILE.parent))
    group = srdf.getroot().find("group[@name='arm']")
    ET.SubElement(group, 'joint', name='gripper_robotiq_85_left_knuckle_joint')
    path = tmp_path / 'compound.srdf'
    srdf.write(path)
    with pytest.raises(ValueError, match='planning group.*single'):
        selection(lambda d: d['model'].__setitem__('semantic', str(path)))


@pytest.mark.parametrize('owner,key,value', [
    ('cell', 'robot_prim_path', '/World/robot-name'),
    ('cell', 'robot_prim_path', '//World/Robot'),
    ('model', 'articulation_root', 'root-joint'),
])
def test_binding_paths_require_prim_identifiers(selection, owner, key, value):
    with pytest.raises(ValueError, match=key):
        selection(lambda d: d[owner]['isaac'].__setitem__(key, value))
