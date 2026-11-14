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


@pytest.mark.parametrize('profile', ['operational_profile.yaml', 'rb5_850_profile.yaml'])
def test_selected_model_in_moveit(profile, tmp_path):
    from arm_cell_bringup.runtime import load_profile

    config = load_profile(ROOT / 'ros2_ws/src/bringup/config' / profile)
    model = config['model_configuration']
    spec = importlib.util.spec_from_file_location(
        'motion_description', ROOT / 'ros2_ws/src/motion/launch/robot_description.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fixture = tmp_path / 'selected.json'
    fixture.write_text(json.dumps({
        'configuration': config,
        'urdf': module.load_robot_description(model['urdf'], model['mesh_root']),
        'kinematics': yaml.safe_load(
            Path(config['motion_configuration']['kinematics']).read_text()),
    }))
    subprocess.run([os.environ['ARM_CELL_MODEL_PROBE']], check=True,
                   env=dict(os.environ, ARM_CELL_MODEL_TEST_INPUT=str(fixture)))
