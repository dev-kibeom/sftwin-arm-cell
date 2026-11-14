import json
import os
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'infra/launcher'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/bringup'))
from arm_cell_bringup.runtime import export_runtime_configuration
from launcher import deployments, profile_choices, selected_deployment, export_selection


def test_default_matches_cli_and_preserves_sources(tmp_path):
    before = {p: p.read_bytes() for p in (ROOT / 'ros2_ws/src').rglob('*.yaml')}
    entries = deployments(ROOT)
    assert len(entries) == 2
    assert entries[0]["profile"].name == "operational_profile.yaml"
    entry = entries[0]
    choices = profile_choices(ROOT)
    assert all(entry[key] in choices[key] for key in ('robot', 'tool', 'cell', 'backend'))
    assert selected_deployment(entries, {key: entry[key] for key in choices}) == entry
    result = export_selection(entry, tmp_path / 'gui.json')
    cli = export_runtime_configuration(ROOT / 'ros2_ws/src/bringup/config/operational_profile.yaml', tmp_path / 'cli.json')
    assert json.loads(result.read_text())['configuration'] == json.loads(cli.read_text())['configuration']
    assert all(p.read_bytes() == data for p, data in before.items())


def test_unregistered_combination_is_rejected():
    entries = deployments(ROOT)
    selection = {key: entries[0][key] for key in ('robot', 'tool', 'cell', 'backend')}
    selection['robot'] = '/unsupported.yaml'
    with pytest.raises(ValueError, match='Unsupported'):
        selected_deployment(entries, selection)


def test_new_production_profile_is_discovered_without_ui_change(tmp_path):
    # Existing profile references remain absolute; discovery depends on deployment owners.
    import yaml
    source = ROOT / 'ros2_ws/src/bringup/config/operational_profile.yaml'
    data = yaml.safe_load(source.read_text())
    data['components']['simulation'] = str(ROOT / 'infra/isaac_sim/config/simulation.yaml')
    config = tmp_path / 'ros2_ws/src/bringup/config'
    config.mkdir(parents=True)
    (config / 'another_profile.yaml').write_text(yaml.safe_dump(data))
    assert deployments(tmp_path)[0]['profile'].name == 'another_profile.yaml'


def test_new_deployment_cannot_claim_unsupported_isaac_robot(tmp_path):
    import yaml
    source = ROOT / 'ros2_ws/src/bringup/config/operational_profile.yaml'
    data = yaml.safe_load(source.read_text())
    data['components']['simulation'] = str(ROOT / 'infra/isaac_sim/config/simulation.yaml')
    data['profiles']['robot'] = str(tmp_path / 'unsupported_robot.yaml')
    config = tmp_path / 'ros2_ws/src/bringup/config'
    config.mkdir(parents=True)
    (config / 'unsupported_profile.yaml').write_text(yaml.safe_dump(data))
    assert deployments(tmp_path) == []


@pytest.mark.parametrize('arguments', [[], ['missing.json']])
def test_isaac_preexport_option_rejects_missing_snapshot(tmp_path, arguments):
    import subprocess
    simulator = tmp_path / 'isaac-sim.sh'
    simulator.write_text('#!/bin/sh\necho UNEXPECTED_START\n')
    simulator.chmod(0o755)
    result = subprocess.run(['bash', str(ROOT / 'infra/isaac_sim/scripts/isaac_arm_cell.sh'),
                             '--runtime-config', *arguments],
                            env=dict(os.environ, ISAAC_SIM_ROOT=str(tmp_path)),
                            capture_output=True, text=True)
    assert result.returncode == 2
    assert 'Readable runtime configuration required' in result.stderr
    assert 'UNEXPECTED_START' not in result.stdout


@pytest.mark.parametrize('level', [None, 'debug'])
def test_ros_terminal_script_forwards_existing_launch_options(tmp_path, level):
    import subprocess
    import json
    # Isolate installed ROS setup while exercising the real entry script argv.
    setup = tmp_path / 'shell_setup.bash'
    setup.write_text('source() { :; }\n')
    ros = tmp_path / 'ros2'
    ros.write_text('#!/usr/bin/env python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
    ros.chmod(0o755)
    profile = str(tmp_path / 'snapshot with spaces.json')
    result = subprocess.run(['bash', str(ROOT / 'infra/launcher/start_ros.bash'), profile,
                             *([level] if level else [])],
                            env=dict(os.environ, BASH_ENV=str(setup),
                                     PATH=str(tmp_path) + os.pathsep + os.environ['PATH']),
                            capture_output=True, text=True, check=True)
    assert json.loads(result.stdout) == ['launch', 'arm_cell_bringup', 'arm_cell.launch.py',
                                       f'profile:={profile}', f'log_level:={level or "info"}']


def test_registered_rb5_export_is_isolated_from_default(tmp_path):
    entries = deployments(ROOT)
    rb = next(entry for entry in entries if entry['profile'].name == 'rb5_850_profile.yaml')
    assert rb['validation_status'] == 'live-pending'
    first = json.loads(export_selection(entries[0], tmp_path / 'before.json').read_text())
    selected = json.loads(export_selection(rb, tmp_path / 'rb.json').read_text())['configuration']
    again = json.loads(export_selection(entries[0], tmp_path / 'after.json').read_text())
    assert first == again
    assert selected['robot']['robot_id'] == 'rb5_850'
    assert selected['model_configuration']['robot_id'] == selected['robot']['robot_id']
    assert selected['tool'] == first['configuration']['tool']
    assert selected['motion']['arm_joint_names'] == selected['robot']['motion']['arm_joint_names']
    assert selected['cell'] == first['configuration']['cell']


@pytest.mark.parametrize('robot_profile', ['operational_profile.yaml', 'rb5_850_profile.yaml'])
@pytest.mark.parametrize('actuator,grasp', [('logical','fixed_attach'), ('physics','fixed_attach'), ('physics','physical')])
def test_selected_tool_modes_export_without_changing_robot_or_sources(tmp_path, robot_profile, actuator, grasp):
    entry = next(item for item in deployments(ROOT) if item['profile'].name == robot_profile)
    baseline = json.loads(export_selection(entry, tmp_path/'baseline.json').read_text())['configuration']
    selected = dict(entry, tool_actuator=actuator, tool_grasp=grasp)
    actual = json.loads(export_selection(selected, tmp_path/'selected.json').read_text())['configuration']
    assert actual['tool']['runtime']['actuator'] == actuator
    assert actual['tool']['runtime']['grasp'] == grasp
    assert actual['robot'] == baseline['robot']
    assert actual['home_joint_positions'] == baseline['home_joint_positions']
    assert actual['cell'] == baseline['cell']
    assert json.loads(export_selection(entry, tmp_path/'again.json').read_text())['configuration'] == baseline


def test_unsupported_tool_modes_rejected_by_configuration_producer(tmp_path):
    entry = dict(deployments(ROOT)[0], tool_actuator='logical', tool_grasp='physical')
    with pytest.raises(ValueError, match='combination'):
        export_selection(entry, tmp_path/'invalid.json')


def test_exported_configuration_cannot_change_tool_selection_at_load(tmp_path):
    from arm_cell_bringup.runtime import load_profile
    path = export_selection(deployments(ROOT)[0], tmp_path/'runtime.json')
    with pytest.raises(ValueError, match='Re-export'):
        load_profile(path, tool_actuator='physics', tool_grasp='physical')
