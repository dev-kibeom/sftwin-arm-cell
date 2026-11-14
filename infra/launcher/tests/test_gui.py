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
from PySide6.QtWidgets import QApplication
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


@pytest.mark.parametrize('robot', ['m0609', 'rb5_850'])
def test_snapshot_same_for_both_duplicate_guard_and_graceful_close(app, window, monkeypatch, robot):
    selector = window.selectors['robot']
    selected = str(launcher.ROOT / f'ros2_ws/src/motion/config/robots/{robot}.yaml')
    selector.setCurrentIndex(selector.findData(selected))
    if robot == 'rb5_850':
        assert '라이브 검증 전' in window.status.text()
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
    if robot == 'rb5_850':
        assert '라이브 검증 전' in window.status.text()
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
    ('Logical Snap', ['logical', 'fixed_attach']),
    ('Physics Snap', ['physics', 'fixed_attach']),
    ('Physics Real', ['physics', 'physical']),
])
def test_tool_mode_presets_export_one_shared_selection(window, monkeypatch, label, pair):
    combo = window.tool_mode
    assert combo.currentText() == 'Logical Snap'
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
            'launcher_modes': [dict(label='Selected Tool Snap',
                combination=['logical', 'fixed_attach'], description='Selected Tool description')],
        }
    }
    path = tmp_path/'selected-tool.yaml'
    path.write_text(yaml.safe_dump(profile))
    tool = window.selectors['tool']
    tool.addItem('Selected Tool', str(path))
    tool.setCurrentIndex(tool.findData(str(path)))
    assert window.tool_mode.currentText() == 'Selected Tool Snap'
    assert window.tool_mode.count() == 1
    assert window.tool_mode.toolTip() == 'Selected Tool description'
    assert not window.isaac.isEnabled()  # No registered deployment claims this test Tool.
