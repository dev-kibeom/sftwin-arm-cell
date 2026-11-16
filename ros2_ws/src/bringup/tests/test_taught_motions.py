import json
from pathlib import Path

import yaml

from arm_cell_bringup.runtime import load_profile

PROFILE = Path(__file__).resolve().parents[1] / 'config/operational_profile.yaml'


def test_named_motion_profiles_reach_runtime():
    config = load_profile(PROFILE)
    motions = json.loads(config['motion']['taught_motions_json'])
    assert motions['home']['target'] == 'home'
    assert motions['return_material']['target'] == 'tray_return'
    assert 0 < motions['home']['velocity_scale'] < 1
    assert motions['home']['velocity_scale'] != motions['return_material']['velocity_scale']


def test_taught_motion_reference_is_resolved_from_profile(tmp_path):
    profile = yaml.safe_load(PROFILE.read_text())
    simulation = PROFILE.parents[4] / 'infra/isaac_sim/config/simulation.yaml'
    profile['components']['simulation'] = str(simulation)
    motions = {'motions': {'standby': {
        'joint_positions': [0.1] * 6,
        'velocity_scale': 0.25,
        'acceleration_scale': 0.15,
    }}}
    (tmp_path / 'motions.yaml').write_text(yaml.safe_dump(motions))
    profile['taught_motion_configuration'] = 'motions.yaml'
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    config = load_profile(path)
    assert json.loads(config['motion']['taught_motions_json']) == motions['motions']
