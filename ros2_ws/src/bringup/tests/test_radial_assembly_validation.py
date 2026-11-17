"""Static assembly and taught-joint bounds apply without registering an HCR deployment."""
from pathlib import Path

import pytest

from arm_cell_bringup.assembly_model import load_assembly_model
from arm_cell_bringup.configured_supply import radial_tool_model
from arm_cell_bringup.model_validation import validate_assembly_configuration, joint_pose, planning_limits

ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize('mutation', ['binding', 'combination', 'primary', 'gain', 'speed'])
def test_invalid_radial_assembly_rejected(mutation):
    config = load_assembly_model('m0609', 'hand_e_c10')
    if mutation == 'binding':
        config['model_configuration']['isaac']['prismatic_joints'].pop()
    elif mutation == 'combination':
        config['tool']['runtime']['supported_combinations'] = [['logical', 'fixed_attach']]
    elif mutation == 'primary':
        config['tool']['runtime']['primary_joint'] = 'joint_1'
    elif mutation == 'gain':
        config['tool']['runtime']['drive_stiffness'] = float('nan')
    else:
        config['tool']['runtime']['max_velocity_fraction'] = 1.01
    with pytest.raises(ValueError):
        radial_tool_model(config)


@pytest.mark.parametrize('mutation', ['missing_pose', 'joint_limit', 'nonfinite'])
def test_taught_joint_inputs_use_production_limits(mutation):
    config = load_assembly_model('m0609', 'hand_e_c10')
    _, joints, arm = validate_assembly_configuration(config)
    overrides = planning_limits(config['robot']['joint_limits'], arm, joints)
    pose = list(config['home_joint_positions'])
    if mutation == 'missing_pose':
        pose = None
    elif mutation == 'joint_limit':
        pose[0] = 100.
    else:
        pose[0] = float('nan')
    with pytest.raises(ValueError):
        joint_pose(pose, 'Configured taught pose', arm, joints, overrides)


@pytest.mark.parametrize('tool_id', ['hand_e_c10', 'hand_e_c10_long'])
def test_hcr14_radial_static_assembly_has_no_registered_deployment(tool_id):
    config = load_assembly_model('hcr14', tool_id)
    assert config['model_configuration']['robot_id'] == 'hcr14'
    assert config['tool']['runtime']['mode_id'] == 'physical_real'
    assert 'cell' not in config
    assert 'mission' not in config
    from arm_cell_bringup.runtime import read, resolve
    for profile in (ROOT / 'ros2_ws/src/bringup/config').glob('*_profile.yaml'):
        source = read(profile)
        if 'model_configuration' in source:
            model = read(resolve(source['model_configuration'], profile.parent))
            assert model['model_id'] != f'hcr14_{tool_id}'
