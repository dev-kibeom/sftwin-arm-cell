"""Profile selection and host process control; Bringup owns configuration semantics."""
import codecs
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from PySide6.QtCore import QLockFile, QTimer
from PySide6.QtWidgets import (QApplication, QComboBox, QFormLayout, QHBoxLayout,
                              QLabel, QMainWindow, QPlainTextEdit, QPushButton,
                              QVBoxLayout, QWidget)


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/bringup'))
from arm_cell_bringup.runtime import DEFAULT_PROFILES, export_runtime_configuration, read, resolve


OWNERS = {
    'robot': 'ros2_ws/src/motion/config/robots',
    'tool': 'ros2_ws/src/motion/config/tools',
    'cell': 'ros2_ws/src/bringup/config/cells',
}


def deployments(root=ROOT):
    """Only complete production deployments register runnable combinations.

    The current host entry point implements the existing Isaac composition.
    Validation fixtures and arbitrary cross-products are never deployments.
    """
    entries = []
    for path in sorted((root / 'ros2_ws/src/bringup/config').glob('*_profile.yaml')):
        data = read(path)
        if data.get('schema_version') != 2 or 'simulation' not in data.get('components', {}):
            continue
        backend = resolve(data['components']['simulation'], path.parent)
        simulation = read(backend)
        if simulation.get('planning_profile') != 'isaac_planning.yaml':
            continue
        owners = {key: str(resolve(data['profiles'][key], path.parent)) for key in OWNERS}
        # isaac_arm_cell_startup.py currently builds only the established M0609
        # Cell. A new deployment alone cannot add simulator implementation.
        if any(owners[key] != str(resolve(reference, path.parent))
               for key, reference in DEFAULT_PROFILES.items()):
            continue
        entries.append(dict(profile=path, backend=str(backend), **owners))
    return entries


def profile_choices(root=ROOT):
    result = {key: [str(path.resolve()) for path in sorted((root / directory).glob('*.yaml'))]
              for key, directory in OWNERS.items()}
    result['backend'] = sorted({entry['backend'] for entry in deployments(root)})
    return result


def selected_deployment(entries, selection):
    matches = [entry for entry in entries if all(entry[key] == value for key, value in selection.items())]
    if len(matches) != 1:
        raise ValueError('Unsupported or ambiguous combination: select a registered production Profile')
    return matches[0]


def export_selection(entry, destination):
    # No parallel configuration engine or edited copies of source profiles.
    return export_runtime_configuration(entry['profile'], destination)


class Launcher(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ARM Cell Launcher')
        self.resize(760, 520)
        self.processes = {}
        self.streams = {}
        self.decoders = {}
        self.stopping = False
        self.closing = False
        self.phase = 0
        self.run = None
        self.error = ''
        # One host simulation uses fixed ports even across repository checkouts.
        self.lock = QLockFile(str(Path(tempfile.gettempdir()) / f'arm-cell-launcher-{os.getuid()}.lock'))
        self.lock.setStaleLockTime(0)
        self.entries = deployments()
        panel = QWidget()
        self.setCentralWidget(panel)
        layout = QVBoxLayout(panel)
        form = QFormLayout()
        layout.addLayout(form)
        self.selectors = {}
        for key, paths in profile_choices().items():
            combo = QComboBox()
            for path in paths:
                combo.addItem(('Isaac / ' if key == 'backend' else '') + Path(path).stem, path)
            self.selectors[key] = combo
            combo.currentIndexChanged.connect(self.validate)
            form.addRow(key.title(), combo)
        buttons = QHBoxLayout()
        layout.addLayout(buttons)
        self.isaac = QPushButton('Isaac 시작')
        self.ros = QPushButton('ROS 2 시작 (Isaac 초기화 확인 후)')
        self.stop = QPushButton('모두 종료')
        for button in (self.isaac, self.ros, self.stop):
            buttons.addWidget(button)
        self.isaac.clicked.connect(self.start_isaac)
        self.ros.clicked.connect(self.start_ros)
        self.stop.clicked.connect(self.begin_stop)
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMaximumBlockCount(2000)
        layout.addWidget(self.output)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(100)
        self.validate()

    def selection(self):
        return selected_deployment(self.entries, {key: combo.currentData() for key, combo in self.selectors.items()})

    def validate(self):
        valid = True
        try:
            entry = self.selection()
            message = f"Profile: {entry['profile'].name}"
        except ValueError as exc:
            valid, message = False, str(exc)
        active = bool(self.processes) or self.stopping
        for combo in self.selectors.values():
            combo.setEnabled(not active)
        self.isaac.setEnabled(valid and not active)
        self.ros.setEnabled('isaac' in self.processes and 'ros' not in self.processes and not self.stopping)
        self.stop.setEnabled(active and not self.stopping)
        states = ', '.join(f'{name}: 프로세스 실행 중' for name in self.processes)
        self.status.setText(self.error or ('종료 중…' if self.stopping else states or message))

    def spawn(self, name, command, environment):
        process = subprocess.Popen(command, cwd=ROOT, env=environment,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.processes[name] = process
        self.streams[name] = process.stdout
        os.set_blocking(process.stdout.fileno(), False)
        self.decoders[name] = codecs.getincrementaldecoder('utf-8')(errors='replace')

    def read_output(self, name):
        try:
            chunk = os.read(self.streams[name].fileno(), 65536)
        except BlockingIOError:
            return False
        text = self.decoders[name].decode(chunk, final=not chunk)
        if text:
            self.output.appendPlainText(f'[{name}] ' + text.rstrip())
        return bool(chunk)

    def start_isaac(self):
        if self.processes or self.stopping:
            return
        self.error = ''
        if not self.lock.tryLock(0):
            self.error = '다른 ARM Cell Launcher가 실행 중입니다.'
            self.validate()
            return
        try:
            self.run = tempfile.TemporaryDirectory(prefix='arm-cell-launcher-')
            runtime = export_selection(self.selection(), Path(self.run.name) / 'runtime_configuration.json')
            # Both consumers retain this one snapshot until all owned groups exit.
            self.environment = dict(os.environ, SFTWIN_PROJECT_ROOT=str(ROOT),
                                    ARM_CELL_PROFILE=str(runtime),
                                    ARM_CELL_RUNTIME_CONFIG=str(runtime), PYTHONUNBUFFERED='1')
            self.runtime = runtime
            self.spawn('isaac', ['bash', str(ROOT / 'infra/isaac_sim/scripts/isaac_arm_cell.sh'),
                                 '--runtime-config', str(runtime)], self.environment)
            self.output.appendPlainText(f'임시 구성: {runtime}\nIsaac 초기화 후 ROS 2를 시작하세요. 프로세스 실행은 시스템 준비 완료를 뜻하지 않습니다.')
        except Exception as exc:
            self.error = str(exc)
            self.begin_stop()
        self.validate()

    def start_ros(self):
        if 'isaac' not in self.processes or 'ros' in self.processes or self.stopping:
            return
        try:
            self.spawn('ros', ['bash', str(ROOT / 'infra/launcher/start_ros.bash'), str(self.runtime)], self.environment)
        except Exception as exc:
            self.error = str(exc)
            self.begin_stop()
        self.validate()

    def signal_groups(self, sig):
        for process in self.processes.values():
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass

    def begin_stop(self):
        if not self.stopping:
            self.stopping = True
            self.phase = 0
            self.signal_groups(signal.SIGINT)
            self.deadline = time.monotonic() + 15
        self.validate()

    def poll(self):
        for name in self.streams:
            self.read_output(name)
        if not self.stopping:
            for name, process in self.processes.items():
                code = process.poll()
                if code is not None:
                    self.error = f'{name} 프로세스 종료 (코드 {code}). GUI 출력을 확인하세요.'
                    self.begin_stop()
                    break
        if self.stopping:
            # Keep group IDs even after leaders exit: grandchildren may still
            # need TERM/KILL. Never release the lock while a group remains.
            alive = False
            for process in self.processes.values():
                process.poll()
                try:
                    os.killpg(process.pid, 0)
                    alive = True
                except ProcessLookupError:
                    pass
            if not alive:
                for name, stream in self.streams.items():
                    while self.read_output(name):
                        pass
                    stream.close()
                self.processes.clear()
                self.streams.clear()
                self.decoders.clear()
                if self.run is not None:
                    self.run.cleanup()
                    self.run = None
                self.lock.unlock()
                self.stopping = False
                if self.closing:
                    self.close()
            elif time.monotonic() >= self.deadline:
                self.signal_groups(signal.SIGTERM if self.phase == 0 else signal.SIGKILL)
                self.phase += 1
                self.deadline = time.monotonic() + 5
        self.validate()

    def closeEvent(self, event):
        if self.processes or self.stopping:
            self.closing = True
            self.begin_stop()
            event.ignore()
        else:
            event.accept()


def main():
    app = QApplication(sys.argv)
    window = Launcher()
    window.show()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    return app.exec()


if __name__ == '__main__':
    sys.exit(main())
