"""Prove that external consumers need data, not Bringup or ROS Python code."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest

from arm_cell_bringup.runtime import export_runtime_configuration, load_profile, read

ROOT = Path(__file__).resolve().parents[4]
CELL = ROOT / 'infra/isaac_sim/scripts/pnp_cnc_cell'
PROFILE = ROOT / 'ros2_ws/src/bringup/config/operational_profile.yaml'


@pytest.mark.parametrize('filename', ['runtime.json', 'runtime.snapshot'])
def test_json_configuration_does_not_use_yaml_parser(tmp_path, monkeypatch, filename):
    path = tmp_path / filename
    path.write_text('  \n{"enabled": true, "values": [0.1, 0.2]}')

    def reject_yaml(_):
        pytest.fail('JSON snapshots must bypass the YAML parser')

    monkeypatch.setattr('arm_cell_bringup.runtime.yaml.safe_load', reject_yaml)
    assert read(path) == {'enabled': True, 'values': [0.1, 0.2]}


@pytest.mark.parametrize('text', ['enabled: true\nvalues: [0.1, 0.2]',
                                  '{enabled: true, values: [0.1, 0.2]}'])
def test_configuration_preserves_yaml_support(tmp_path, text):
    path = tmp_path / 'owner.yaml'
    path.write_text(text)
    assert read(path) == {'enabled': True, 'values': [0.1, 0.2]}


@pytest.mark.parametrize('text', ['[]', '[1, 2]', 'null', 'value'])
def test_configuration_requires_mapping(tmp_path, text):
    path = tmp_path / 'invalid.snapshot'
    path.write_text(text)
    with pytest.raises(ValueError, match='must be a mapping'):
        read(path)


def test_runtime_snapshot_reloads_are_isolated_and_verify_current_file(tmp_path):
    configuration = {
        'orchestration': {'mission_target_id': 'TestCube'},
        '_mission_recipe': {'target_id': 'TestCube', 'schema_version': 1},
        'motion': {'positions': [0.1, 0.2]},
    }
    document = {
        'kind': 'arm-cell-runtime-configuration', 'schema_version': 1,
        'configuration': configuration,
        'configuration_sha256': hashlib.sha256(
            json.dumps(configuration, sort_keys=True, allow_nan=False).encode()).hexdigest(),
    }
    path = tmp_path / 'runtime.snapshot'
    path.write_text(json.dumps(document))
    first = load_profile(path)
    first['motion']['positions'][0] = 99
    assert load_profile(path)['motion']['positions'] == [0.1, 0.2]

    document['configuration']['motion']['positions'][0] = 0.3
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='checksum mismatch'):
        load_profile(path)

    document['configuration_sha256'] = hashlib.sha256(
        json.dumps(configuration, sort_keys=True, allow_nan=False).encode()).hexdigest()
    path.write_text(json.dumps(document))
    assert load_profile(path)['motion']['positions'] == [0.3, 0.2]

    document['schema_version'] = 2
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match='Unsupported runtime configuration schema'):
        load_profile(path)


@pytest.mark.parametrize('constant', ['NaN', 'Infinity', '-Infinity'])
def test_json_configuration_rejects_nonfinite_constants(tmp_path, constant):
    path = tmp_path / 'runtime.snapshot'
    path.write_text('{"value": ' + constant + '}')
    with pytest.raises(ValueError, match='finite'):
        read(path)


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
    # Keep ROS Python dependencies needed by the exporter's model validation.
    # The launcher adds Bringup only for export; the simulator must not inherit it.
    environment['PYTHONPATH'] = os.pathsep.join(
        entry for entry in environment.get('PYTHONPATH', '').split(os.pathsep)
        if entry and 'bringup' not in entry)
    environment['ISAAC_SIM_ROOT'] = str(launcher_root)
    environment['ARM_CELL_RUNTIME_CONFIG'] = str(tmp_path / 'runtime.json')
    environment['ARM_CELL_PROFILE'] = str(PROFILE)
    result = subprocess.run(['bash', str(ROOT / 'infra/isaac_sim/scripts/isaac_arm_cell.sh')], env=environment, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'ISAAC_DATA_ONLY' in result.stdout


def test_launcher_consumes_exported_motion_selection_without_ros_python(tmp_path):
    import os
    import shlex

    snapshot = export_runtime_configuration(PROFILE, tmp_path / 'runtime.json')
    launcher_root = tmp_path / 'isaac'
    launcher_root.mkdir()
    launcher = launcher_root / 'isaac-sim.sh'
    code = """import json, os
from pathlib import Path
config = json.loads(Path(os.environ['ARM_CELL_RUNTIME_CONFIG']).read_text())['configuration']
assert config['motion']['planning_group'] == config['model_configuration']['planning_group']
assert config['motion']['model_id'] == config['model_configuration']['model_id']
for key in ('wrap_equivalent_joints', 'place_ik_seed_joints'):
    assert config['motion'][key] == config['robot']['joint_policy'][key]
print('SELECTED_MOTION_CONFIGURATION')
"""
    launcher.write_text('#!/bin/sh\nexec python3 -I -c ' + shlex.quote(code) + '\n')
    launcher.chmod(0o755)
    environment = dict(os.environ)
    environment.pop('PYTHONPATH', None)
    environment['ISAAC_SIM_ROOT'] = str(launcher_root)
    result = subprocess.run(
        ['bash', str(ROOT / 'infra/isaac_sim/scripts/isaac_arm_cell.sh'),
         '--runtime-config', str(snapshot)],
        env=environment, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert 'SELECTED_MOTION_CONFIGURATION' in result.stdout
