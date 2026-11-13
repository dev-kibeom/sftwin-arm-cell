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
    monkeypatch.setattr(widget, 'record_root', lambda: tmp_path)
    yield widget
    if widget.processes:
        widget.signal_groups(signal.SIGKILL)
        for process in widget.processes.values():
            process.wait(timeout=5)
    for stream in widget.streams.values():
        stream.close()
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


WAIT = [sys.executable, '-c', 'import time; print("started", flush=True); time.sleep(100)']


def test_snapshot_same_for_both_duplicate_guard_and_graceful_close(app, window, monkeypatch):
    calls = fake_spawn(window, monkeypatch, {'isaac': WAIT, 'ros': WAIT})
    window.start_ros()
    assert not calls
    window.start_isaac()
    assert not window.isaac.isEnabled()
    assert all(not combo.isEnabled() for combo in window.selectors.values())
    window.start_isaac()
    window.start_ros()
    window.start_ros()
    assert [call[0] for call in calls] == ['isaac', 'ros']
    assert calls[1][1][-1] == str(window.runtime)
    assert calls[0][2]['ARM_CELL_PROFILE'] == str(window.runtime)
    assert calls[0][2] == calls[1][2]
    # Exercise existing Isaac CLI using this snapshot and a stand-in simulator.
    fake = window.run / 'isaac'
    fake.mkdir()
    script = fake / 'isaac-sim.sh'
    script.write_text('#!/bin/sh\nexit 0\n')
    script.chmod(0o755)
    import subprocess
    subprocess.run(['bash', str(launcher.ROOT / 'infra/isaac_sim/scripts/isaac_arm_cell.sh')],
                   env=dict(calls[0][2], ISAAC_SIM_ROOT=str(fake)), check=True)
    isaac = json.loads(Path(calls[0][2]['ARM_CELL_RUNTIME_CONFIG']).read_text())
    ros = json.loads(window.runtime.read_text())
    assert isaac['configuration_sha256'] == ros['configuration_sha256']
    processes = list(window.processes.values())
    window.show()
    window.close()
    assert window.stopping
    until(app, window, lambda: not window.processes)
    assert all(process.poll() is not None for process in processes)
    assert not window.isVisible()


def test_unexpected_exit_stops_peer_and_reports_error(app, window, monkeypatch):
    fake_spawn(window, monkeypatch, {'isaac': WAIT, 'ros': [sys.executable, '-c', 'raise SystemExit(7)']})
    window.start_isaac()
    window.start_ros()
    until(app, window, lambda: not window.processes)
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
    assert not window.lock.isLocked()


def test_stop_escalates_for_unresponsive_process(app, window, monkeypatch):
    command = [sys.executable, '-c', 'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); print("ready",flush=True); time.sleep(100)']
    fake_spawn(window, monkeypatch, {'isaac': command})
    window.start_isaac()
    until(app, window, lambda: 'ready' in window.output.toPlainText())
    window.begin_stop()
    window.deadline = 0
    window.poll()
    assert window.phase == 1
    window.deadline = 0
    window.poll()
    until(app, window, lambda: not window.processes)
    assert window.isaac.isEnabled()


def test_exited_leader_does_not_orphan_child(app, window, monkeypatch):
    code = ('import subprocess,sys; '
            'child=subprocess.Popen([sys.executable,"-c","import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); time.sleep(100)"]); '
            'print(child.pid,flush=True)')
    fake_spawn(window, monkeypatch, {'isaac': [sys.executable, '-c', code]})
    window.start_isaac()
    until(app, window, lambda: window.stopping)
    window.deadline = 0
    window.poll()
    until(app, window, lambda: not window.processes)
