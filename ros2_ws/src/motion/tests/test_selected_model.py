"""Real MoveIt model/IK verification through the selected owner configuration."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/bringup'))


def run_model(profile, tmp_path, linear_replay=None):
    from arm_cell_bringup.runtime import load_profile

    config = load_profile(ROOT / 'ros2_ws/src/bringup/config' / profile)
    model = config['model_configuration']
    spec = importlib.util.spec_from_file_location(
        'motion_description', ROOT / 'ros2_ws/src/motion/launch/robot_description.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fixture = tmp_path / 'selected.json'
    data = {
        'configuration': config,
        'urdf': module.load_robot_description(model['urdf'], model['mesh_root']),
        'kinematics': yaml.safe_load(
            Path(config['motion_configuration']['kinematics']).read_text()),
    }
    if linear_replay is not None:
        data['linear_replay'] = linear_replay
    fixture.write_text(json.dumps(data))
    subprocess.run([os.environ['ARM_CELL_MODEL_PROBE']], check=True,
                   env=dict(os.environ, ARM_CELL_MODEL_TEST_INPUT=str(fixture)))


@pytest.mark.parametrize('profile', ['operational_profile.yaml', 'rb5_850_profile.yaml'])
def test_selected_model_in_moveit(profile, tmp_path):
    run_model(profile, tmp_path)


@pytest.mark.parametrize('profile', ['tape_m0609_c10_profile.yaml', 'tape_m0609_c10_long_profile.yaml'])
def test_tape_teaching_in_moveit(profile, tmp_path, monkeypatch):
    monkeypatch.setenv('ARM_CELL_MODEL_PROBE', os.environ['ARM_CELL_TEACHING_MODEL_PROBE'])
    run_model(profile, tmp_path)


def test_tape_place_descent_from_recorded_start(tmp_path, monkeypatch):
    monkeypatch.setenv('ARM_CELL_MODEL_PROBE', os.environ['ARM_CELL_TEACHING_MODEL_PROBE'])
    # Failed tape_transfer waypoint 6 from the 2026-11-20 07:48 run.
    run_model('tape_m0609_c10_long_profile.yaml', tmp_path, linear_replay={
        'start_joint_positions': [-0.8758, 0.5103, 0.2762, 0.0004, 2.3556, 2.266],
        'target_tcp_pose': [0.3, -0.35, 0.2975, 1., 0., 0., 0.],
    })
