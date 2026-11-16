"""Teaching inputs remain bounded model experiments and fail before Isaac launch."""
from pathlib import Path

import pytest
import yaml

from arm_cell_bringup.model_experiment import load_model_experiment
from arm_cell_bringup.runtime import read, resolve

ROOT = Path(__file__).resolve().parents[4]
PROFILE = ROOT / 'ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10.yaml'


@pytest.mark.parametrize('mutation', ['missing_pose', 'joint_limit', 'nonfinite', 'duration', 'fixture'])
def test_invalid_teaching_and_fixture_rejected(tmp_path, mutation):
    source = read(PROFILE)
    for key in ('robot', 'tool', 'model_configuration', 'motion_configuration'):
        source[key] = str(resolve(source[key], PROFILE.parent))
    teaching = read(resolve(source['internal_grasp_experiment'], PROFILE.parent))
    if mutation == 'missing_pose':
        teaching.pop('insert')
    elif mutation == 'joint_limit':
        teaching['transfer'][0] = 100.
    elif mutation == 'nonfinite':
        teaching['approach'][0] = float('nan')
    elif mutation == 'duration':
        teaching['move_duration_s'] = 0.
    else:
        source['internal_grasp_fixture']['insertion_depth_m'] = -.1
    path = tmp_path/'teaching.yaml'
    path.write_text(yaml.safe_dump(teaching))
    source['internal_grasp_experiment'] = str(path)
    path = tmp_path/'experiment.yaml'
    path.write_text(yaml.safe_dump(source))
    with pytest.raises(ValueError):
        load_model_experiment(path)


def test_hcr14_model_experiments_preserved():
    for suffix in ('', '_long'):
        config = load_model_experiment(PROFILE.with_name(f'hcr14_hand_e_c10{suffix}.yaml'))
        assert 'internal_grasp_experiment' not in config
        assert config['execution_scope'] == 'model_only'
