"""Prove that external consumers need data, not Bringup or ROS Python code."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest

from arm_cell_bringup.runtime import export_runtime_configuration

ROOT = Path(__file__).resolve().parents[4]
CELL = ROOT / 'infra/isaac_sim/scripts/m0609_cell'
PROFILE = ROOT / 'ros2_ws/src/bringup/config/operational_profile.yaml'


def test_export_is_self_contained_and_readable_without_ros_or_bringup(tmp_path):
    path = export_runtime_configuration(PROFILE, tmp_path / 'runtime.json')
    document = json.loads(path.read_text())
    config = document['configuration']
    assert 'recipe_directory' not in config['orchestration']
    assert len(config['home_joint_positions']) == 6
    assert config['_mission_recipe']['target_id'] == config['orchestration']['mission_target_id']
    # Isolated interpreter ignores PYTHONPATH. Only the consumer module is added.
    code = '''
import sys
sys.path.insert(0, sys.argv[1])
from runtime_configuration import load_runtime_configuration
config = load_runtime_configuration(sys.argv[2])
assert config['material_definitions']['TestCube']['shape'] == 'box'
assert not any(name.startswith(('arm_cell_bringup', 'arm_cell_configuration', 'ament_index')) for name in sys.modules)
'''
    result = subprocess.run([sys.executable, '-I', '-c', code, str(CELL), str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_export_rejects_tampered_configuration(tmp_path):
    path = export_runtime_configuration(PROFILE, tmp_path / 'runtime.json')
    document = json.loads(path.read_text())
    document['configuration']['motion']['object_to_grasp_tcp_translation'] = [1, 2, 3]
    path.write_text(json.dumps(document))
    code = 'import sys; sys.path.insert(0, sys.argv[1]); from runtime_configuration import load_runtime_configuration; load_runtime_configuration(sys.argv[2])'
    result = subprocess.run([sys.executable, '-I', '-c', code, str(CELL), str(path)], capture_output=True, text=True)
    assert result.returncode != 0
    assert 'checksum mismatch' in result.stderr


def test_export_validation_embeds_selected_recipe(tmp_path):
    validation = PROFILE.parent / 'pnp_validation_profile.yaml'
    path = export_runtime_configuration(validation, tmp_path / 'validation.json')
    config = json.loads(path.read_text())['configuration']
    assert config['_mission_recipe']['target_id'] == 'TestCube'
    assert config['_mission_recipe']['pick']['approach_distance_m'] == 0.05


def test_launcher_exports_data_before_starting_isaac(tmp_path):
    import os
    import shlex

    launcher_root = tmp_path / 'isaac'
    launcher_root.mkdir()
    fake_launcher = launcher_root / 'isaac-sim.sh'
    code = """import json, os
from pathlib import Path
assert 'bringup' not in os.environ.get('PYTHONPATH', '')
path = Path(os.environ['ARM_CELL_RUNTIME_CONFIG'])
assert json.loads(path.read_text())['kind'] == 'arm-cell-runtime-configuration'
print('ISAAC_DATA_ONLY')
"""
    fake_launcher.write_text('#!/bin/sh\nexec python3 -I -c ' + shlex.quote(code) + '\n')
    fake_launcher.chmod(0o755)
    environment = dict(os.environ)
    environment.pop('PYTHONPATH', None)
    environment['ISAAC_SIM_ROOT'] = str(launcher_root)
    environment['ARM_CELL_RUNTIME_CONFIG'] = str(tmp_path / 'runtime.json')
    environment['ARM_CELL_PROFILE'] = str(PROFILE)
    result = subprocess.run(['bash', str(ROOT / 'infra/isaac_sim/scripts/isaac_arm_cell.sh')], env=environment, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'ISAAC_DATA_ONLY' in result.stdout
