"""Real Qt event loop with controlled host processes; no simulator claim."""
import json
import os
from pathlib import Path
import signal
import sys
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pytest
from PySide6.QtCore import QLockFile
from PySide6.QtWidgets import QApplication, QDoubleSpinBox, QFormLayout
import launcher


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def window(app, tmp_path, monkeypatch):
    widget = launcher.Launcher()
    widget.timer.stop()
    widget.lock = QLockFile(str(tmp_path / 'launcher.lock'))
    monkeypatch.setattr(launcher.tempfile, 'tempdir', str(tmp_path))
    def fake_terminal(name, slave, master):
        code = """import os,sys,signal
fd = int(sys.argv[1])
signal.signal(signal.SIGUSR1, lambda *_: os.write(fd, bytes([3])))
os.write(fd, b'fake-window-id\\n')
with open(sys.argv[2], 'wb', buffering=0) as stream:
    while True:
        try:
            data = os.read(fd, 65536)
        except OSError:
            break
        if not data:
            break
        stream.write(data)
"""
        return [sys.executable, '-c', code, str(master), str(tmp_path / f'{name}.txt')]
    widget.test_output_root = tmp_path
    monkeypatch.setattr(launcher, 'terminal_command', fake_terminal)
    yield widget
    owned = [*widget.processes.values(), *widget.terminals.values()]
    launcher.signal_processes(owned, signal.SIGKILL)
    for process in owned:
        process.wait(timeout=5)
    widget.terminals.clear()
    if widget.run is not None:
        widget.run.cleanup()
    widget.processes.clear()
    widget.stopping = False
    widget.lock.unlock()
    widget.close()


def until(app, widget, predicate):
    deadline = time.monotonic() + 5
    while not predicate():
        app.processEvents()
        widget.poll()
        if time.monotonic() >= deadline:
            pytest.fail('process state did not converge: ' + widget.status.text())
        time.sleep(.01)


def fake_spawn(widget, monkeypatch, commands):
    original = widget.spawn
    calls = []
    def spawn(name, command, env):
        calls.append((name, command, env.copy()))
        original(name, commands[name], env)
    monkeypatch.setattr(widget, 'spawn', spawn)
    return calls


def terminal_text(widget, name):
    path = widget.test_output_root / f'{name}.txt'
    return path.read_text(errors='replace') if path.exists() else ''


WAIT = [sys.executable, '-c', 'import time; print("started", flush=True); time.sleep(100)']


@pytest.mark.parametrize('robot', ['m0609', 'rb5_850', 'hcr14'])
def test_snapshot_same_for_both_duplicate_guard_and_graceful_close(app, window, monkeypatch, robot):
    selector = window.selectors['robot']
    selected = str(launcher.ROOT / f'ros2_ws/src/motion/config/robots/{robot}.yaml')
    selector.setCurrentIndex(selector.findData(selected))
    if robot == 'rb5_850':
        assert '라이브 검증 전' in window.status.text()
    if robot == 'hcr14':
        assert '실험적 모델' in window.status.text()
    settings = window.component_settings
    field = settings.findChild(QDoubleSpinBox, 'motion.planning_velocity_scaling_factor')
    field.setValue(0.05)
    calls = fake_spawn(window, monkeypatch, {'isaac': WAIT, 'ros': WAIT})
    exports = []
    original_export = launcher.export_selection
    def export(entry, destination):
        exports.append(destination)
        return original_export(entry, destination)
    monkeypatch.setattr(launcher, 'export_selection', export)
    window.start_ros()
    assert not calls
    window.start_isaac()
    assert not window.isaac.isEnabled()
    assert all(not combo.isEnabled() for combo in window.selectors.values())
    assert not settings.isEnabled()
    window.start_isaac()
    window.start_ros()
    window.start_ros()
    assert [call[0] for call in calls] == ['isaac', 'ros']
    assert exports == [window.runtime]
    assert calls[1][1][-2:] == [str(window.runtime), 'info']
    assert calls[0][2]['ARM_CELL_PROFILE'] == str(window.runtime)
    assert calls[0][2] == calls[1][2]
    # Exercise existing Isaac CLI using this snapshot and a stand-in simulator.
    run = Path(window.run.name)
    fake = run / 'isaac'
    fake.mkdir()
    script = fake / 'isaac-sim.sh'
    script.write_text('#!/bin/sh\nexit 0\n')
    script.chmod(0o755)
    python_shim = fake / 'python3'
    python_shim.write_text('#!/bin/sh\necho unexpected-export >&2\nexit 93\n')
    python_shim.chmod(0o755)
    import subprocess
    before = window.runtime.read_bytes()
    stamp = window.runtime.stat().st_mtime_ns
    subprocess.run(['bash', str(launcher.ROOT / 'infra/isaac_sim/scripts/isaac_arm_cell.sh'), '--runtime-config', str(window.runtime)],
                   env=dict(calls[0][2], ISAAC_SIM_ROOT=str(fake), PATH=str(fake) + os.pathsep + os.environ['PATH']), check=True)
    assert window.runtime.read_bytes() == before
    assert window.runtime.stat().st_mtime_ns == stamp
    assert Path(calls[0][2]['ARM_CELL_RUNTIME_CONFIG']) == window.runtime
    assert sorted(path.name for path in run.iterdir()) == ['isaac', 'runtime_configuration.json']
    isaac = json.loads(Path(calls[0][2]['ARM_CELL_RUNTIME_CONFIG']).read_text())
    ros = json.loads(window.runtime.read_text())
    assert isaac['configuration_sha256'] == ros['configuration_sha256']
    assert isaac['configuration']['robot']['robot_id'] == robot
    assert isaac['configuration']['motion']['planning_velocity_scaling_factor'] == 0.05
    if robot == 'rb5_850':
        assert '라이브 검증 전' in window.status.text()
    if robot == 'hcr14':
        assert '실험적 모델' in window.status.text()
    processes = list(window.processes.values())
    window.show()
    window.close()
    assert window.stopping
    until(app, window, lambda: not window.processes)
    assert all(process.poll() is not None for process in processes)
    assert not window.isVisible()
    assert not run.exists()


def test_unexpected_exit_stops_peer_and_reports_error(app, window, monkeypatch):
    fake_spawn(window, monkeypatch, {'isaac': WAIT, 'ros': [sys.executable, '-c', 'raise SystemExit(7)']})
    window.start_isaac()
    window.start_ros()
    runtime = window.runtime
    until(app, window, lambda: not window.processes)
    assert not runtime.parent.exists()
    assert '코드 7' in window.status.text()
    assert window.isaac.isEnabled()


def test_lock_rejects_second_launcher(window, monkeypatch):
    other = QLockFile(window.lock.fileName())
    assert other.tryLock(0)
    window.start_isaac()
    assert not window.processes
    assert '다른' in window.status.text()
    other.unlock()


def test_spawn_error_releases_lock_and_can_retry(app, window, monkeypatch):
    monkeypatch.setattr(window, 'spawn', lambda *_: (_ for _ in ()).throw(OSError('spawn failed')))
    window.start_isaac()
    until(app, window, lambda: not window.stopping)
    assert 'spawn failed' in window.status.text()
    assert window.run is None
    assert not window.lock.isLocked()


def test_stop_escalates_for_unresponsive_process(app, window, monkeypatch):
    command = [sys.executable, '-c', 'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); print("ready",flush=True); time.sleep(100)']
    fake_spawn(window, monkeypatch, {'isaac': command})
    window.start_isaac()
    until(app, window, lambda: 'ready' in terminal_text(window, 'isaac'))
    window.begin_stop()
    window.deadline = 0
    window.poll()
    assert window.phase == 1
    window.deadline = 0
    window.poll()
    until(app, window, lambda: not window.processes)
    assert window.isaac.isEnabled()


def test_exited_leader_does_not_orphan_child(app, window, monkeypatch):
    child_code = ('import signal,time; '
                  'signal.signal(signal.SIGINT, signal.SIG_IGN); '
                  'signal.signal(signal.SIGHUP, signal.SIG_IGN); '
                  'print("CHILD_READY",flush=True); time.sleep(100)')
    code = ('import subprocess,sys; '
            f'child=subprocess.Popen([sys.executable,"-c",{child_code!r}],stdout=subprocess.PIPE); '
            'assert child.stdout.readline().strip() == b"CHILD_READY"; '
            'print(child.pid,flush=True)')
    fake_spawn(window, monkeypatch, {'isaac': [sys.executable, '-c', code]})
    window.start_isaac()
    until(app, window, lambda: window.stopping)
    runtime = window.runtime
    assert runtime.exists()
    window.deadline = 0
    window.poll()
    until(app, window, lambda: not window.processes)
    assert not runtime.parent.exists()


def test_output_uses_terminal_without_gui_or_disk_logs(app, window, monkeypatch):
    records = launcher.ROOT / '.run_records'
    before = {p: p.stat().st_mtime_ns for p in records.rglob('*')}
    command = [sys.executable, '-u', '-c', 'import sys,time; print("stdout marker"); print("stderr marker",file=sys.stderr); time.sleep(100)']
    fake_spawn(window, monkeypatch, {'isaac': command})
    window.start_isaac()
    run = Path(window.run.name)
    assert not hasattr(window, 'output')
    until(app, window, lambda: 'stderr marker' in terminal_text(window, 'isaac'))
    assert 'stdout marker' in terminal_text(window, 'isaac')
    assert [path.name for path in run.iterdir()] == ['runtime_configuration.json']
    assert {p: p.stat().st_mtime_ns for p in records.rglob('*')} == before
    window.begin_stop()
    assert window.runtime.exists()
    until(app, window, lambda: not window.processes)
    assert not run.exists()


def test_export_failure_removes_partial_snapshot(app, window, monkeypatch):
    paths = []
    def fail(entry, destination):
        destination.write_text('partial')
        paths.append(destination)
        raise ValueError('export failed')
    monkeypatch.setattr(launcher, 'export_selection', fail)
    window.start_isaac()
    until(app, window, lambda: not window.stopping)
    assert not paths[0].parent.exists()
    assert not window.lock.isLocked()
    assert 'export failed' in window.status.text()


def test_terminal_backpressure_does_not_deadlock(app, window, monkeypatch):
    command = [sys.executable, '-u', '-c', 'import time; print("x" * 200000); print("OUTPUT COMPLETE"); time.sleep(100)']
    fake_spawn(window, monkeypatch, {'isaac': command})
    window.start_isaac()
    until(app, window, lambda: 'OUTPUT COMPLETE' in terminal_text(window, 'isaac'))
    window.begin_stop()
    until(app, window, lambda: not window.processes)


def test_repeated_status_refresh_keeps_gui_process_alive():
    # A binding refcount regression can abort Python rather than raise an
    # exception, so keep this sustained refresh check in a separate process.
    import subprocess
    code = '''
import sys
sys.path.insert(0, sys.argv[1])
from PySide6.QtWidgets import QApplication
from launcher import Launcher
app = QApplication([])
window = Launcher()
window.timer.stop()
for _ in range(2000):
    window.poll()
    app.processEvents()
window.close()
print('REFRESH_SURVIVED', flush=True)
'''
    result = subprocess.run([sys.executable, '-c', code, str(Path(launcher.__file__).parent)],
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert 'REFRESH_SURVIVED' in result.stdout


@pytest.mark.parametrize('debug,expected', [(False, 'info'), (True, 'debug')])
def test_ros_debug_option_uses_existing_bringup_mode(app, window, monkeypatch, debug, expected):
    calls = fake_spawn(window, monkeypatch, {'isaac': WAIT, 'ros': WAIT})
    window.start_isaac()
    assert window.debug.isEnabled()
    window.debug.setChecked(debug)
    window.start_ros()
    assert calls[-1][1][-1] == expected
    assert not window.debug.isEnabled()
    window.begin_stop()
    until(app, window, lambda: not window.stopping)


def test_terminal_close_stops_consumers_before_snapshot_cleanup(app, window, monkeypatch):
    fake_spawn(window, monkeypatch, {'isaac': WAIT, 'ros': WAIT})
    window.start_isaac()
    window.start_ros()
    runtime = window.runtime
    processes = list(window.processes.values())
    window.terminals['isaac'].terminate()
    until(app, window, lambda: not window.stopping and not window.processes)
    assert all(p.poll() is not None for p in processes)
    assert not window.terminals
    assert not runtime.parent.exists()
    assert 'isaac' in window.status.text() and '종료' in window.status.text()


def test_terminal_start_failure_cleans_snapshot(app, window, monkeypatch):
    monkeypatch.setattr(launcher, 'terminal_command', lambda *_: (_ for _ in ()).throw(FileNotFoundError('xterm missing')))
    window.start_isaac()
    until(app, window, lambda: not window.stopping)
    assert not window.processes and not window.terminals
    assert window.run is None
    assert 'xterm missing' in window.status.text()


def test_terminal_ctrl_c_reaches_actual_runtime(app, window, monkeypatch):
    command = [sys.executable, '-u', '-c', 'import os,time; print("TTY_READY", os.isatty(0), os.tcgetpgrp(0) == os.getpgrp(), flush=True); time.sleep(100)']
    fake_spawn(window, monkeypatch, {'isaac': command})
    window.start_isaac()
    until(app, window, lambda: 'TTY_READY True True' in terminal_text(window, 'isaac'))
    runtime = window.processes['isaac']
    window.terminals['isaac'].send_signal(signal.SIGUSR1)
    until(app, window, lambda: not window.stopping and not window.processes)
    assert runtime.returncode == -signal.SIGINT


@pytest.mark.parametrize('label,pair', [
    ('논리 판정 + 고정', ['logical', 'fixed_attach']),
    ('물리 판정 + 고정', ['physics', 'fixed_attach']),
    ('물리 파지', ['physics', 'physical']),
])
def test_tool_mode_presets_export_one_shared_selection(window, monkeypatch, label, pair):
    combo = window.tool_mode
    assert combo.currentText() == '논리 판정 + 고정'
    combo.setCurrentIndex(combo.findText(label))
    assert combo.currentData() == pair
    assert window.isaac.isEnabled()
    calls = fake_spawn(window, monkeypatch, {'isaac': WAIT, 'ros': WAIT})
    window.start_isaac()
    assert not combo.isEnabled()
    selected = json.loads(window.runtime.read_text())['configuration']['tool']['runtime']
    assert [selected['actuator'], selected['grasp']] == pair
    window.start_ros()
    assert calls[0][2]['ARM_CELL_RUNTIME_CONFIG'] == calls[1][2]['ARM_CELL_RUNTIME_CONFIG']


def test_mode_choices_follow_selected_tool_profile(window, tmp_path):
    import yaml
    profile = {
        'runtime': {
            'actuator': 'logical', 'grasp': 'fixed_attach',
            'supported_combinations': [['logical', 'fixed_attach']],
            'launcher_modes': [dict(label='논리 판정 + 고정',
                combination=['logical', 'fixed_attach'], description='Selected Tool description')],
        }
    }
    path = tmp_path/'selected-tool.yaml'
    path.write_text(yaml.safe_dump(profile))
    tool = window.selectors['tool']
    tool.addItem('Selected Tool', str(path))
    tool.setCurrentIndex(tool.findData(str(path)))
    assert window.tool_mode.currentText() == '논리 판정 + 고정'
    assert window.tool_mode.count() == 1
    assert window.tool_mode.toolTip() == 'Selected Tool description'
    assert not window.isaac.isEnabled()  # No registered deployment claims this test Tool.


def test_invalid_mode_label_disables_start_until_profile_is_valid(window, monkeypatch):
    selected = window.selectors['tool'].currentData()
    original_read = launcher.read

    def read_invalid_mode(path):
        profile = original_read(path)
        if path == selected:
            profile['runtime']['launcher_modes'][0]['label'] = '새 모드'
        return profile

    with monkeypatch.context() as patch:
        patch.setattr(launcher, 'read', read_invalid_mode)
        window.refresh_tool_modes()
        assert '파지 모드' in window.status.text()
        assert window.tool_mode.count() == 0
        assert not window.isaac.isEnabled()

    window.refresh_tool_modes()
    assert window.tool_mode.currentText() == '논리 판정 + 고정'
    assert window.isaac.isEnabled()


def test_grasp_parameters_export_reset_and_mode_scope(window, tmp_path):
    window.parameter_edits['move_duration_s'].setValue(0.75)
    entry = window.selection()
    destination = launcher.export_selection(entry, tmp_path / 'logical.json')
    document = json.loads(destination.read_text())
    assert document['configuration']['tool']['runtime']['move_duration_s'] == 0.75
    window.reset_parameters.click()
    assert window.parameter_edits['move_duration_s'].value() == 0.4
    window.tool_mode.setCurrentText('물리 파지')
    assert 'move_duration_s' not in window.parameter_edits
    window.parameter_edits['physics.speed_fraction'].setValue(0.25)
    window.parameter_edits['physics.contact_material.dynamic_friction'].setValue(2.)
    assert not window.isaac.isEnabled()
    window.parameter_edits['physics.contact_material.dynamic_friction'].setValue(0.5)
    destination = launcher.export_selection(window.selection(), tmp_path / 'physics.json')
    runtime = json.loads(destination.read_text())['configuration']['tool']['runtime']
    assert runtime['physics']['speed_fraction'] == 0.25
    assert runtime['mode_id'] == 'physical_real'
    # Export does not rewrite the selected owner profile.
    assert launcher.read(window.selectors['tool'].currentData())['runtime']['physics']['speed_fraction'] == 0.5


def test_grasp_parameter_producer_rejects_invalid_overrides(window, tmp_path):
    entry = window.selection()
    for parameters in ({'physics.speed_fraction': 0.3}, {'move_duration_s': float('nan')},
                       {'move_duration_s': -1}, {'primary_joint': 1}):
        with pytest.raises(ValueError):
            launcher.export_selection(dict(entry, tool_parameters=parameters), tmp_path / 'bad.json')



def test_launcher_has_only_normal_start_controls_for_c10(app,window,monkeypatch):
    entry=next(item for item in window.entries if item['profile'].stem=='tape_m0609_c10_long_profile')
    for key,combo in window.selectors.items():
        combo.setCurrentIndex(combo.findData(entry[key]))
    assert window.tool_mode.currentData()==['physics','physical']
    assert window.isaac.isEnabled()
    assert not hasattr(window,'experiment_buttons')
    assert not hasattr(window,'start_model_experiment')
    calls=fake_spawn(window,monkeypatch,{'isaac':WAIT,'ros':WAIT})
    monkeypatch.setattr(launcher,'preparation_command',lambda entry,destination:
        [sys.executable,'-c','from pathlib import Path; import sys; Path(sys.argv[1]).write_text("{}")',str(destination)])
    window.start_isaac()
    assert window.preparing and not calls
    assert '준비 중' in window.status.text()
    until(app,window,lambda:bool(calls))
    assert calls[0][1][1].endswith('isaac_arm_cell.sh')
    assert window.ros.isEnabled()
    window.start_ros()
    assert len(calls)==2
    window.begin_stop()
    until(app,window,lambda:not window.stopping)


def test_launcher_filters_choices_after_cell_change(window, monkeypatch):
    errors = []
    monkeypatch.setattr(sys, 'excepthook', lambda kind, error, tb: errors.append(error))

    def options(combo):
        return [Path(combo.itemData(index)).stem for index in range(combo.count())]

    cells = window.selectors['cell']
    robot = window.selectors['robot']
    tool = window.selectors['tool']
    form = window.centralWidget().layout().itemAt(0).layout()
    for row, widget in enumerate((cells, robot, tool, window.strategy, window.mode_display, window.selectors['backend'])):
        assert form.itemAt(row, QFormLayout.FieldRole).widget() is widget

    arm_index = cells.currentIndex()
    assert set(options(robot)) == {'m0609', 'rb5_850', 'hcr14'}
    assert options(tool) == ['robotiq_2f85']
    assert window.tool_mode.count() == 3
    assert window.selection()['profile'].stem == 'operational_profile'

    window.tool_mode.setCurrentText('물리 판정 + 고정')
    robot.setCurrentIndex(options(robot).index('rb5_850'))
    assert window.selection()['profile'].stem == 'rb5_850_profile'
    assert window.tool_mode.currentText() == '물리 판정 + 고정'

    cells.setCurrentIndex(options(cells).index('tape_roll_cell'))
    assert set(options(robot)) == {'m0609', 'rb5_850', 'hcr14'}
    assert Path(robot.currentData()).stem == 'rb5_850'
    assert set(options(tool)) == {'hand_e_c10', 'hand_e_c10_long'}
    assert window.tool_mode.count() == 1
    assert window.tool_mode.currentText() == '물리 파지'
    assert not window.isaac.isEnabled()

    robot.setCurrentIndex(options(robot).index('m0609'))
    assert window.isaac.isEnabled()
    for suffix in ('_long', ''):
        tool.setCurrentIndex(options(tool).index(f'hand_e_c10{suffix}'))
        entry = window.selection()
        assert entry['profile'].stem == f'tape_m0609_c10{suffix}_profile'
        assert (entry['tool_actuator'], entry['tool_grasp']) == ('physics', 'physical')
        assert 'move_duration_s' not in window.parameter_edits
        assert 'max_velocity_fraction' in window.parameter_edits

    cells.setCurrentIndex(options(cells).index('conveyor_pnp_cell'))
    assert Path(robot.currentData()).stem == 'm0609'
    assert options(tool) == ['suction']
    assert window.selection()['profile'].stem == 'conveyor_pnp_profile'
    assert window.isaac.isEnabled()

    cells.setCurrentIndex(arm_index)
    assert options(tool) == ['robotiq_2f85']
    assert window.tool_mode.count() == 3
    assert window.selection()['profile'].stem == 'operational_profile'
    assert 'move_duration_s' in window.parameter_edits
    assert window.isaac.isEnabled()
    assert errors == []


def test_configured_preparation_can_be_canceled_before_isaac_starts(app, window, monkeypatch):
    entry=next(item for item in window.entries if item['profile'].stem=='tape_m0609_c10_long_profile')
    for key,combo in window.selectors.items():combo.setCurrentIndex(combo.findData(entry[key]))
    monkeypatch.setattr(launcher,'preparation_command',lambda *_:WAIT)
    calls=fake_spawn(window,monkeypatch,{'isaac':WAIT})
    window.start_isaac()
    assert window.preparing and window.stop.isEnabled() and not window.ros.isEnabled()
    app.processEvents()
    window.begin_stop()
    until(app,window,lambda:not window.stopping)
    assert not calls and window.run is None and not window.preparing


def test_configured_preparation_failure_is_visible_and_does_not_start_isaac(app, window, monkeypatch):
    entry=next(item for item in window.entries if item['profile'].stem=='tape_m0609_c10_long_profile')
    for key,combo in window.selectors.items():combo.setCurrentIndex(combo.findData(entry[key]))
    monkeypatch.setattr(launcher,'preparation_command',lambda *_:
        [sys.executable,'-c','print("unreachable teaching",flush=True); raise SystemExit(2)'])
    calls=fake_spawn(window,monkeypatch,{'isaac':WAIT})
    window.start_isaac()
    until(app,window,lambda:window.run is None)
    assert 'unreachable teaching' in window.status.text()
    assert not calls



def test_experiment_tabs_keep_edits_after_stop_and_restore_yaml_defaults(app, window, monkeypatch):
    settings = window.component_settings
    key = 'motion.planning_velocity_scaling_factor'
    assert [settings.tabText(i) for i in range(settings.count())] == ['시뮬레이션', '모션', '비전']
    field = settings.findChild(QDoubleSpinBox, key)
    assert field.value() == 0.1
    field.setValue(0.05)
    assert field.styleSheet()
    fake_spawn(window, monkeypatch, {'isaac': WAIT})
    window.start_isaac()
    until(app, window, lambda: 'started' in terminal_text(window, 'isaac'))
    window.begin_stop()
    until(app, window, lambda: not window.stopping)
    assert settings.isEnabled()
    assert field.value() == 0.05

    window.reset_parameters.click()
    assert settings.findChild(QDoubleSpinBox, key).value() == 0.1
    assert window.selection()['component_parameters'] == {}

    settings.findChild(QDoubleSpinBox, key).setValue(0.05)
    robot = window.selectors['robot']
    rb5 = str(launcher.ROOT / 'ros2_ws/src/motion/config/robots/rb5_850.yaml')
    robot.setCurrentIndex(robot.findData(rb5))
    assert settings.findChild(QDoubleSpinBox, key).value() == 0.1
    settings.findChild(QDoubleSpinBox, key).setValue(0.05)

    reopened = launcher.Launcher()
    try:
        assert reopened.selection()['component_parameters'] == {}
    finally:
        reopened.close()


@pytest.mark.parametrize("start_ros", [True, False])
def test_conveyor_can_prepare_plc_before_ros(app, window, monkeypatch, start_ros):
    assert window.plc_button.isHidden()
    entry = next(item for item in window.entries if item['profile'].stem == 'conveyor_pnp_profile')
    for key, combo in window.selectors.items():
        combo.setCurrentIndex(combo.findData(entry[key]))
    assert window.strategy.text() == '표면 흡착'
    assert window.fixed_mode.text() == '물리 파지'
    assert window.tool_mode.isHidden()
    assert not window.plc_button.isHidden()
    assert not window.plc_button.isEnabled()
    fake_spawn(window, monkeypatch, {'isaac': WAIT, 'ros': WAIT})
    window.start_isaac()
    until(app, window, lambda: 'isaac' in window.processes)
    assert window.plc_button.isEnabled()
    commands = []
    def fake_command(stage, runtime):
        assert runtime == window.runtime
        commands.append(stage)
        return [sys.executable, '-c', 'pass'] if stage == 'probe' else WAIT
    monkeypatch.setattr(launcher.plc_control, 'runtime_command', fake_command)
    monkeypatch.setattr(window.plc, 'inspect', lambda phase: window.plc.service_observed(
        phase, 'ActiveState=active\nInvocationID=existing'))
    window.plc_button.click()
    until(app, window, lambda: window.plc.phase == 'awaiting_ros')
    assert commands == ['probe']
    assert not window.plc_button.isEnabled()
    if start_ros:
        window.start_ros()
        window.poll()
        assert commands == ['probe']
        window.ros_preflight.touch()
        until(app, window, lambda: window.plc.phase == 'adapter')
        assert commands == ['probe', 'adapter']
    window.begin_stop()
    until(app, window, lambda: not window.stopping)


@pytest.mark.parametrize('service_active', [True, False])
def test_plc_retry_preserves_ros_and_releases_only_owned_service(app, window, tmp_path, monkeypatch, service_active):
    entry = next(item for item in window.entries if item['profile'].stem == 'conveyor_pnp_profile')
    for key, combo in window.selectors.items():
        combo.setCurrentIndex(combo.findData(entry[key]))
    monkeypatch.setattr(launcher.plc_control, 'service_command',
                        lambda verb: ['systemctl', verb, '--no-ask-password', launcher.plc_control.UNIT])
    state = tmp_path / 'active'
    if service_active:
        state.touch()
    systemctl = tmp_path / 'systemctl'
    systemctl.write_text("""#!/bin/sh
case "$1" in
show) echo LoadState=loaded; if test -f "$PLC_TEST_DIR/active"; then echo ActiveState=active; echo InvocationID=1234; else echo ActiveState=inactive; fi ;;
start) touch "$PLC_TEST_DIR/active" ;;
stop) rm "$PLC_TEST_DIR/active" ;;
*) exit 2 ;;
esac
""")
    systemctl.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path) + os.pathsep + os.environ['PATH'])
    monkeypatch.setenv('PLC_TEST_DIR', str(tmp_path))
    fake_spawn(window, monkeypatch, {'isaac': WAIT, 'ros': WAIT})
    window.start_isaac()
    until(app, window, lambda: 'isaac' in window.processes)
    window.start_ros()
    window.ros_preflight.touch()
    window.poll()
    ros = window.processes['ros']
    commands = []
    def command(stage, runtime):
        assert runtime == window.runtime
        commands.append(stage)
        if stage == 'probe':
            return [sys.executable, '-c', 'pass']
        if commands.count('adapter') == 1:
            return [sys.executable, '-c', 'raise SystemExit("writer locked")']
        return WAIT
    monkeypatch.setattr(launcher.plc_control, 'runtime_command', command)
    window.plc_button.click()
    window.plc_button.click()
    until(app, window, lambda: window.plc_button.isEnabled())
    assert 'writer locked' in window.status.text()
    assert commands == ['probe', 'adapter']
    assert ros.poll() is None and state.exists()
    window.plc_button.click()
    until(app, window, lambda: commands.count('adapter') == 2)
    window.begin_stop()
    until(app, window, lambda: not window.stopping)
    assert state.exists() is service_active


@pytest.mark.parametrize('use_override', [False, True])
def test_optional_codesys_gui_opens_selected_project_once(window, tmp_path, monkeypatch, use_override):
    assert window.plc_gui.isHidden()
    cell = window.selectors['cell']
    cell.setCurrentIndex(next(i for i in range(cell.count()) if Path(cell.itemData(i)).stem == 'conveyor_pnp_cell'))
    assert not window.plc_gui.isHidden()
    assert not window.plc_gui.isChecked()
    project = Path(__file__).resolve().parents[2] / 'plc' / 'projects' / 'Conveyor.fbsdev'
    window.environment = dict(os.environ)
    window.environment.pop('ARM_CELL_CODESYS_PROJECT', None)
    if use_override:
        project = tmp_path / 'Conveyor.fbsdev'
        project.mkdir()
        window.environment['ARM_CELL_CODESYS_PROJECT'] = str(project)
    opened = []
    class Editor:
        def poll(self):
            return None
    def open_process(args, **kwargs):
        opened.append(args)
        return Editor()
    monkeypatch.setattr(launcher.plc_control.subprocess, 'Popen', open_process)
    window.runtime = tmp_path / 'runtime.json'
    monkeypatch.setattr(window, 'can_start_plc', lambda: True)
    starts = []
    monkeypatch.setattr(window.plc, 'start', lambda *args: starts.append(args))
    window.start_plc()
    assert not opened
    window.plc_gui.setChecked(True)
    window.start_plc()
    window.start_plc()
    assert len(starts) == 3
    assert opened == [['c4-cli', 'standalone-session', str(project)]]


def test_missing_editor_project_keeps_plc_start_available(window, tmp_path, monkeypatch):
    window.environment = dict(os.environ, ARM_CELL_CODESYS_PROJECT=str(tmp_path / 'missing.fbsdev'))
    window.runtime = tmp_path / 'runtime.json'
    window.plc_gui.setChecked(True)
    monkeypatch.setattr(window, 'can_start_plc', lambda: True)
    starts = []
    monkeypatch.setattr(window.plc, 'start', lambda runtime, env: starts.append(runtime))
    window.start_plc()
    assert starts == [window.runtime]
    assert 'missing.fbsdev' in window.plc.editor_status()
