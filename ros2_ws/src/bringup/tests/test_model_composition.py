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
    policy_path = resolve(deployment['motion_configuration'], PROFILE.parent)
    documents['motion_configuration'] = {
        key: str(resolve(value, policy_path.parent))
        for key, value in yaml.safe_load(policy_path.read_text()).items()}
    deployment['motion_configuration'] = str(tmp_path / 'motion_configuration.yaml')
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


def test_motion_consumes_selected_model_and_joint_policy():
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
    assert config['motion']['planning_group'] == config['model_configuration']['planning_group']
    assert config['motion']['model_id'] == config['model_configuration']['model_id']
    for key in ('wrap_equivalent_joints', 'place_ik_seed_joints'):
        assert config['motion'][key] == config['robot']['joint_policy'][key]


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
    (lambda d: d['tool']['contact_links']['pick_retract'].append('missing_link'),
     'contact_links.pick_retract'),
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


def test_selected_joint_policy_reaches_motion(selection):
    def change(d):
        d['robot']['joint_policy']['wrap_equivalent_joints'] = []
        d['robot']['joint_policy']['place_ik_seed_joints'] = ['joint_2']
    config = selection(change)
    assert config['motion']['wrap_equivalent_joints'] == []
    assert config['motion']['place_ik_seed_joints'] == ['joint_2']


def test_selected_assembly_identity_and_group_reach_motion(selection, tmp_path):
    import xacro
    def change(d):
        model = d['model']
        urdf = xacro.process_file(model['urdf'], mappings={'mesh_root': model['mesh_root']}).toxml()
        semantic = Path(model['semantic']).read_text()
        (tmp_path / 'selected.urdf').write_text(urdf.replace(model['model_id'], 'selected_assembly'))
        (tmp_path / 'selected.srdf').write_text(
            semantic.replace(model['model_id'], 'selected_assembly').replace('"arm"', '"selected_group"'))
        model.update(model_id='selected_assembly', planning_group='selected_group',
                     urdf=str(tmp_path / 'selected.urdf'), semantic=str(tmp_path / 'selected.srdf'))
        for key in ('kinematics', 'planning_pipeline'):
            data = yaml.safe_load(Path(d['motion_configuration'][key]).read_text())
            data['selected_group'] = data.pop('arm')
            target = tmp_path / (key + '.yaml')
            target.write_text(yaml.safe_dump(data))
            d['motion_configuration'][key] = str(target)
    config = selection(change)
    assert config['motion']['model_id'] == 'selected_assembly'
    assert config['motion']['planning_group'] == 'selected_group'


def test_production_contacts_are_forwarded_independently_of_validation(selection):
    def change(d):
        d['tool']['contact_links']['payload_touch_links'] = ['sf_grasp_tcp']
        d['tool']['contact_links']['pick_retract'] = ['base_link', 'sf_grasp_tcp']
        # Use another existing Cell object, so composition checks the same owners.
        scene = yaml.safe_load(Path(d['cell']['static_scene']).read_text())
        d['cell']['contact_objects']['pick_retract'] = next(
            obj['id'] for obj in scene['objects'] if obj['id'] != 'amr_tray')
    config = selection(change)
    assert config['motion']['payload_touch_links'] == ['sf_grasp_tcp']
    assert config['motion']['pick_retract_contact_links'] == ['base_link', 'sf_grasp_tcp']
    assert (config['motion']['pick_retract_contact_object'] ==
            config['cell']['contact_objects']['pick_retract'])
    assert 'validation_grasp_contact_links' not in config['motion']
    assert (config['tool']['validation_motion'] ==
            load_profile(PROFILE)['tool']['validation_motion'])
    assert (config['tool']['validation_planning_scene']['touch_links'] !=
            config['motion']['payload_touch_links'])


@pytest.mark.parametrize('profile', ['operational_profile.yaml', 'pnp_validation_profile.yaml'])
def test_export_preserves_production_contact_owners(tmp_path, profile):
    from arm_cell_bringup.runtime import export_runtime_configuration
    path = export_runtime_configuration(PROFILE.parent / profile, tmp_path / 'runtime.json')
    config = load_profile(path)
    for parameter, owner in (
        ('payload_touch_links', config['tool']['contact_links']['payload_touch_links']),
        ('pick_retract_contact_links', config['tool']['contact_links']['pick_retract']),
        ('pick_retract_contact_object', config['cell']['contact_objects']['pick_retract']),
    ):
        assert config['motion'][parameter] == owner


@pytest.mark.parametrize('profile', ['operational_profile.yaml', 'pnp_validation_profile.yaml'])
def test_pick_clearance_objects_preserve_existing_rail_selection(tmp_path, profile):
    from arm_cell_bringup.runtime import export_runtime_configuration
    path = export_runtime_configuration(PROFILE.parent / profile, tmp_path / 'runtime.json')
    config = load_profile(path)
    selected = config['motion']['pick_clearance_objects']
    assert selected == config['cell']['collision_objects']['pick_clearance']
    assert selected == ['amr_tray_rail_left', 'amr_tray_rail_right',
                        'amr_tray_rail_front', 'amr_tray_rail_rear']
    # The precheck previously selected all present objects with this prefix.
    scene_path = (config['cell']['static_scene'] if profile == 'operational_profile.yaml'
                  else config['cell']['validation_planning_scene']['static_scene_file'])
    ids = {obj['id'] for obj in yaml.safe_load(Path(scene_path).read_text())['objects']}
    assert ids & set(selected) == {name for name in ids if name.startswith('amr_tray_rail_')}


def test_pick_clearance_objects_follow_selected_cell(selection):
    def change(d):
        d['cell']['collision_objects']['pick_clearance'] = ['amr_tray']
    config = selection(change)
    assert config['motion']['pick_clearance_objects'] == ['amr_tray']


@pytest.mark.parametrize('scales', [
    {'unknown_joint': 2.75}, {'joint_2': 0}, {'joint_2': -1},
    {'joint_2': float('nan')}, {'joint_2': True}, {'joint_2': '2.75'}, [],
])
def test_robot_drive_scales_are_validated_at_composition(selection, scales):
    with pytest.raises(ValueError, match='Robot Isaac joint drive stiffness'):
        selection(lambda documents: documents['robot'].update(
            isaac={'joint_drive_stiffness_scales': scales}))


def test_tool_grasp_collision_links_preserve_robotiq_geometry_selection():
    """Replacing the old prefix selector must retain every collision-bearing Tool link."""
    import xml.etree.ElementTree as ET
    import xacro
    config = load_profile(PROFILE)
    model = config['model_configuration']
    document = xacro.process_file(str(resolve(model['urdf'], PROFILE.parent)))
    robot = ET.fromstring(document.toxml())
    previous = {link.attrib['name'] for link in robot.findall('link')
                if link.attrib['name'].startswith('gripper_') and link.find('collision') is not None}
    assert set(config['tool']['grasp_pose_collision_links']) == previous
    assert config['motion']['tool_grasp_collision_links'] == config['tool']['grasp_pose_collision_links']


@pytest.mark.parametrize('profile_name', ['operational_profile.yaml', 'rb5_850_profile.yaml'])
def test_physics_tool_export_preserves_robot_and_derives_linkage(tmp_path, profile_name):
    from arm_cell_bringup.runtime import load_profile, export_runtime_configuration
    import json
    config_root = Path(__file__).resolve().parents[1] / 'config'
    baseline = load_profile(config_root / profile_name)
    tool = yaml.safe_load((config_root.parents[1] / 'motion/config/tools/robotiq_2f85.yaml').read_text())
    tool['runtime']['actuator'] = 'physics'
    tool['runtime']['grasp'] = 'physical'
    selected_tool = tmp_path / 'tool.yaml'
    selected_tool.write_text(yaml.safe_dump(tool))
    profile = yaml.safe_load((config_root / profile_name).read_text())
    profile['profiles']['tool'] = str(selected_tool)
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    selected = load_profile(path)
    assert selected['robot'] == baseline['robot']
    assert selected['home_joint_positions'] == baseline['home_joint_positions']
    joints = selected['tool']['runtime']['joint_model']
    primary = selected['tool']['runtime']['physics']['primary_joint']
    assert all(name not in selected['robot']['motion']['arm_joint_names'] for name in joints)
    assert {value['mimic']['joint'] for value in joints.values() if 'mimic' in value} == {primary}
    assert sum('mimic' not in value for value in joints.values()) == 1
    exported = export_runtime_configuration(path, tmp_path / 'runtime.json')
    assert json.loads(exported.read_text())['configuration']['tool']['runtime'] == selected['tool']['runtime']
