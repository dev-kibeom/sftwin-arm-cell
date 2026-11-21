"""Profile selection and host process control; Bringup owns configuration semantics."""
import os
import json
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import time

from PySide6.QtCore import QLockFile, QSignalBlocker, QTimer, Qt
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QFileDialog, QFormLayout, QHBoxLayout,
                              QLabel, QMainWindow, QPushButton, QDoubleSpinBox, QGroupBox,
                              QStackedWidget, QVBoxLayout, QWidget)


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/bringup'))
from arm_cell_bringup.tool_modes import tool_mode_choices
from arm_cell_bringup.tool_parameters import available_fields, parameter_value, apply_tool_parameters
from arm_cell_bringup.runtime import export_runtime_configuration, read, resolve
if __package__:
    from .codesys_project import DEFAULT_PROJECT
    from . import plc_control
    from .component_settings import ComponentSettings
else:
    from codesys_project import DEFAULT_PROJECT
    import plc_control
    from component_settings import ComponentSettings


PROFILE_OWNERS = ('cell', 'robot', 'tool')


def deployments(root=ROOT):
    """Complete deployments register runnable combinations, including labelled experiments.

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
        owners = {key: str(resolve(data['profiles'][key], path.parent)) for key in PROFILE_OWNERS}
        # A deployment registers one concrete Assembly and its owner profiles.
        # Discovery checks identity; export performs full model validation.
        model = read(resolve(data['model_configuration'], path.parent))
        try:
            if any(model[f'{key}_id'] != read(owners[key])[f'{key}_id']
                   for key in ('robot', 'tool')):
                continue
        except (OSError, KeyError):
            continue
        status = data.get('launcher', {}).get('validation_status', 'baseline')
        entries.append(dict(profile=path, backend=str(backend), validation_status=status, **owners))
    return sorted(entries, key=lambda entry: (
        entry['profile'].name != 'operational_profile.yaml',
        entry['validation_status'] == 'experimental',
    ))


def selected_deployment(entries, selection):
    matches = [entry for entry in entries if all(entry[key] == value for key, value in selection.items())]
    if len(matches) != 1:
        raise ValueError('Unsupported or ambiguous combination: select a registered production Profile')
    return matches[0]


def validation_note(entry):
    status = entry['validation_status']
    message = ''
    if status == 'experimental':
        message += ' — 실험적 모델 · 단순화 형상 · 실물 일치 미검증'
    if status == 'live-verified':
        message += ' — 기본 설정 Isaac 라이브 확인'
        if entry.get('tool_parameters') or entry.get('component_parameters'):
            message += ' · 변경 설정 미검증'
    elif entry['tool_actuator'] == 'physics' or status == 'live-pending':
        message += ' — Isaac 라이브 검증 전 · 사용자 테스트용'
    return message


def export_selection(entry, destination):
    # No parallel configuration engine or edited copies of source profiles.
    return export_runtime_configuration(
        entry['profile'], destination, tool_actuator=entry.get('tool_actuator'),
        tool_grasp=entry.get('tool_grasp'), tool_parameters=entry.get('tool_parameters'),
        component_parameters=entry.get('component_parameters'),
    )


def preparation_command(entry, destination):
    command = [sys.executable, str(ROOT/'ros2_ws/src/bringup/scripts/export_runtime_configuration'),
               '--profile', str(entry['profile']), '--output', str(destination),
               '--tool-parameters-json', json.dumps(entry.get('tool_parameters', {})),
               '--component-parameters-json', json.dumps(entry.get('component_parameters', {}))]
    for option, key in [('--tool-actuator','tool_actuator'),('--tool-grasp','tool_grasp')]:
        if entry.get(key) is not None:command.extend([option,entry[key]])
    return command


def terminal_command(name, slave, master):
    terminal = shutil.which('xterm')
    if terminal is None:
        raise FileNotFoundError('별도 실행 터미널을 위해 xterm을 설치하세요: sudo apt-get install xterm')
    return [terminal, '-T', f'ARM Cell — {name}', f'-S{os.ttyname(slave)}/{master}']


def signal_processes(processes, sig):
    for process in processes:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass


def groups_alive(processes):
    alive = False
    for process in processes:
        process.poll()
        try:
            os.killpg(process.pid, 0)
            alive = True
        except ProcessLookupError:
            pass
    return alive


class Launcher(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ARM Cell Launcher')
        self.resize(760, 240)
        self.plc = plc_control.ConveyorPLC(self)
        self.conveyor = False
        self.ros_preflight = None
        self.processes = {}
        self.terminals = {}
        self.terminal_stop = False
        self.initialize_dir = None
        self.initialize_phase = None
        self.stopping = False
        self.closing = False
        self.phase = 0
        self.run = None
        self.error = ''
        self.pending_start = None
        self.preparation_output = b''
        self.preparation_started_at = 0.
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
        for key in (*PROFILE_OWNERS, 'backend'):
            combo = QComboBox()
            self.selectors[key] = combo
            form.addRow(key.title(), combo)
            if key in ('robot', 'backend'):
                combo.currentIndexChanged.connect(self.validate)
            else:
                combo.currentIndexChanged.connect(
                    lambda _index, changed=key: self.refresh_deployment_choices(changed))
        self.tool_mode = QComboBox()
        self.strategy = QLabel()
        form.insertRow(form.rowCount() - 1, '파지 전략', self.strategy)
        self.fixed_mode = QLabel()
        self.mode_display = QStackedWidget()
        self.mode_display.addWidget(self.tool_mode)
        self.mode_display.addWidget(self.fixed_mode)
        form.insertRow(form.rowCount() - 1, '파지 모드', self.mode_display)
        self.tool_mode.currentIndexChanged.connect(self.refresh_parameters)
        self.mode_pairs = []
        self.parameter_edits = {}
        self.parameter_panel = QWidget()
        self.parameter_layout = QVBoxLayout(self.parameter_panel)
        self.component_settings = ComponentSettings(self.parameter_panel)
        layout.addWidget(self.component_settings)
        layout.addWidget(QLabel('탭의 변경값은 이번 실행에 적용되며 YAML에 저장되지 않습니다.'))
        self.reset_parameters = QPushButton('기본값 복원')
        self.reset_parameters.clicked.connect(self.restore_parameters)
        layout.addWidget(self.reset_parameters)
        self.resize(760, 650)
        self.debug = QCheckBox('ROS 2 디버그 모드')
        layout.addWidget(self.debug)
        self.plc_gui = QCheckBox('PLC 시작 시 CODESYS GUI 열기')
        self.plc_project = Path(os.environ.get('ARM_CELL_CODESYS_PROJECT') or DEFAULT_PROJECT).expanduser()
        plc_row = QHBoxLayout()
        layout.addLayout(plc_row)
        plc_row.addWidget(self.plc_gui)
        self.plc_project_button = QPushButton(f'프로젝트 선택… · {self.plc_project.name}')
        self.plc_project_button.setToolTip(str(self.plc_project))
        self.plc_project_button.clicked.connect(self.choose_plc_project)
        plc_row.addWidget(self.plc_project_button)
        buttons = QHBoxLayout()
        layout.addLayout(buttons)
        self.isaac = QPushButton('Isaac 시작')
        self.ros = QPushButton('ROS 2 시작')
        self.plc_button = QPushButton('PLC 시작')
        self.stop = QPushButton('모두 종료')
        for button in (self.isaac, self.ros, self.plc_button, self.stop):
            buttons.addWidget(button)
        self.isaac.clicked.connect(self.start_isaac)
        self.ros.clicked.connect(self.start_ros)
        self.plc_button.clicked.connect(self.start_plc)
        self.stop.clicked.connect(self.begin_stop)
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        hint = QLabel('라이브 테스트 세션에 연결하려면 Isaac 초기화 후 ROS 2를 시작하세요.')
        layout.addWidget(hint)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.refresh_deployment_choices()
        self.timer.start(100)

    def refresh_deployment_choices(self, changed=None):
        candidates = self.entries
        previous_tool = self.selectors['tool'].currentData()
        refresh_choices = changed is None
        for key, combo in self.selectors.items():
            if refresh_choices and (key != 'robot' or changed is None):
                current = combo.currentData()
                entries = self.entries if key == 'robot' else candidates
                paths = dict.fromkeys(entry[key] for entry in entries)
                with QSignalBlocker(combo):
                    combo.clear()
                    for path in paths:
                        label = read(path).get('display_name', Path(path).stem)
                        combo.addItem(('Isaac / ' if key == 'backend' else '') + label, path)
                    index = combo.findData(current)
                    if index >= 0:
                        combo.setCurrentIndex(index)
            if key in ('cell', 'tool'):
                candidates = [entry for entry in candidates if entry[key] == combo.currentData()]
            refresh_choices = refresh_choices or key == changed

        cell = self.selectors['cell'].currentData()
        self.conveyor = bool(cell and read(cell).get('conveyor', {}).get('plc_profile'))
        if changed == 'tool' or previous_tool != self.selectors['tool'].currentData():
            self.refresh_tool_modes()
        else:
            self.validate()

    def refresh_tool_modes(self):
        tool = self.selectors['tool'].currentData()
        profile = read(tool) if tool is not None else None
        runtime = profile['runtime'] if profile is not None else {}
        self.mode_error = ''
        try:
            choices = tool_mode_choices(profile) if profile is not None else []
        except ValueError as exc:
            self.mode_error = str(exc)
            choices = []
        labels = {'surface_suction': '표면 흡착', 'internal_radial': '내경 파지', 'box_exterior': '외측 파지'}
        strategies = runtime.get('grasp_strategies', [])
        self.strategy.setText(', '.join(labels.get(key, key) for key in strategies) or '—')
        self.mode_pairs = [choice['combination'] for choice in choices]
        self.tool_mode.blockSignals(True)
        self.tool_mode.clear()
        for choice in choices:
            self.tool_mode.addItem(choice['label'], choice['combination'])
            self.tool_mode.setItemData(self.tool_mode.count() - 1, choice.get('description', ''), Qt.ToolTipRole)
        default = [runtime.get('actuator', 'logical'), runtime.get('grasp', 'fixed_attach')]
        self.tool_mode.setCurrentIndex(self.tool_mode.findData(default))
        self.tool_mode.blockSignals(False)
        self.fixed_mode.setText(self.tool_mode.currentText())
        self.fixed_mode.setToolTip(self.tool_mode.currentData(Qt.ToolTipRole) or '')
        self.mode_display.setCurrentWidget(self.tool_mode if len(choices) > 1 else self.fixed_mode)
        self.refresh_parameters()

    def refresh_parameters(self):
        while self.parameter_layout.count():
            item = self.parameter_layout.takeAt(0)
            item.widget().deleteLater()
        self.parameter_edits = {}
        tool = self.selectors['tool'].currentData()
        if tool is None:
            self.validate()
            return
        runtime = read(tool)['runtime']
        pair = self.tool_mode.currentData()
        if pair:
            runtime = dict(runtime, actuator=pair[0], grasp=pair[1])
        basic = QGroupBox('기본 설정 — 이번 실행에 적용')
        advanced = QGroupBox('고급 설정 — 시뮬레이션 튜닝')
        advanced.setCheckable(True)
        advanced.setChecked(False)
        forms = [QFormLayout(basic), QFormLayout(advanced)]
        for path, label, minimum, maximum, is_advanced, _ in available_fields(runtime):
            edit = QDoubleSpinBox()
            edit.setDecimals(6)
            edit.setRange(minimum, maximum)
            default = parameter_value(runtime, path)
            edit.setValue(default)
            edit.setToolTip(f'{path}\n기본값: {default} | 범위: {minimum}–{maximum}')
            edit.valueChanged.connect(lambda value, widget=edit, initial=default: widget.setStyleSheet(
                'font-weight: bold; color: #b06000;' if value != initial else ''))
            edit.valueChanged.connect(self.validate)
            forms[int(is_advanced)].addRow(label, edit)
            self.parameter_edits[path] = edit
        self.parameter_layout.addWidget(basic)
        if 'physics.grip_force_n' in self.parameter_edits:
            force_hint = QLabel(
                '실행 시작 시 파지력 기본값을 초기화합니다. '
                '일반 PICK에서는 자재별 GripForcePolicy가 계산한 작업 명령의 파지력이 우선합니다. '
                '이 값으로 작업 파지력을 고정할 수는 없습니다.')
            force_hint.setWordWrap(True)
            self.parameter_layout.addWidget(force_hint)
        self.parameter_layout.addWidget(advanced)
        # Collapsed advanced fields remain active settings, but are hidden.
        def show_advanced(checked):
            for row in range(forms[1].rowCount()):
                forms[1].setRowVisible(row, checked)
        advanced.toggled.connect(show_advanced)
        show_advanced(False)
        self.validate()

    def restore_parameters(self):
        self.component_settings.load_profile(self.component_settings.profile)
        self.refresh_parameters()

    def selection(self):
        choices = {key: combo.currentData() for key, combo in self.selectors.items()}
        try:
            entry = selected_deployment(self.entries, choices)
        except ValueError:
            if self.component_settings.profile is not None:
                self.component_settings.load_profile(None)
            raise
        if entry['profile'] != self.component_settings.profile:
            self.component_settings.load_profile(entry['profile'])
        pair = self.tool_mode.currentData()
        if self.mode_error:
            raise ValueError(self.mode_error)
        if pair not in self.mode_pairs:
            raise ValueError('Unsupported Tool actuator/grasp combination')
        runtime = read(self.selectors['tool'].currentData())['runtime']
        parameters = {path: edit.value() for path, edit in self.parameter_edits.items()
                      if edit.value() != parameter_value(runtime, path)}
        runtime.update(actuator=pair[0], grasp=pair[1])
        apply_tool_parameters(runtime, parameters)
        return dict(entry, tool_actuator=pair[0], tool_grasp=pair[1], tool_parameters=parameters,
                    component_parameters=self.component_settings.overrides())

    def validate(self):
        self.tool_mode.setToolTip(self.tool_mode.currentData(Qt.ToolTipRole) or '')
        valid = True
        try:
            entry = self.selection()
            message = f"Profile: {entry['profile'].name} | {self.tool_mode.currentText()}"
            message += validation_note(entry)
        except ValueError as exc:
            valid, message = False, str(exc)
        active = self.run is not None or self.stopping
        for combo in (*self.selectors.values(), self.tool_mode):
            combo.setEnabled(not active)
        self.component_settings.setEnabled(not active)
        self.reset_parameters.setEnabled(not active)
        self.isaac.setEnabled(valid and self.can_start('isaac'))
        self.ros.setEnabled(valid and self.can_start('ros'))
        self.plc_gui.setVisible(self.conveyor)
        self.plc_gui.setEnabled(self.can_start('plc'))
        self.plc_project_button.setVisible(self.conveyor)
        self.plc_project_button.setEnabled(self.plc_gui.isEnabled())
        self.plc_button.setVisible(self.conveyor)
        self.plc_button.setEnabled(valid and self.can_start('plc'))
        self.stop.setEnabled(active and not self.stopping)
        self.debug.setEnabled('ros' not in self.processes and not self.stopping)
        states = ', '.join(f'{name}: 프로세스 실행 중' for name in self.processes)
        if self.pending_start:
            elapsed = int(time.monotonic()-self.preparation_started_at)
            message = f'설정·슬롯 티칭 경로 준비 중… ({elapsed}초) 중지 버튼으로 취소할 수 있습니다.'
        elif self.initialize_phase == "scene":
            message = '초기화 중 · 새 Isaac 장면과 HOME 설정을 기다리는 중'
        elif 'initialize' in self.processes:
            message = '초기화 중 · PLC 응답 대기'
        elif states:
            message = states + ' | ' + message
        if self.plc.message and self.conveyor:
            message = f'{self.plc.message} | {message}'
        if self.conveyor and self.plc_gui.isChecked():
            editor_error = self.plc.editor_status()
            if editor_error:
                message = f'{message} | {editor_error}'
        self.status.setText(self.error or ('종료 중…' if self.stopping else message))

    def spawn(self, name, command, environment):
        # The GUI owns the runtime directly; xterm only displays its PTY.
        # A terminal server process is never used as a proxy for runtime lifetime.
        master, slave = os.openpty()
        try:
            self.terminals[name] = subprocess.Popen(
                terminal_command(name, slave, master), cwd=ROOT, env=environment,
                pass_fds=(master,), start_new_session=True)
            # xterm -S writes its window ID before user input. Consume it before
            # attaching the runtime so it cannot become shell/program input.
            if not select.select([slave], [], [], 5)[0]:
                raise RuntimeError(f'{name} 터미널 시작 응답이 없습니다.')
            if not os.read(slave, 1024).endswith(b'\n'):
                raise RuntimeError(f'{name} 터미널 시작 응답이 올바르지 않습니다.')
            self.processes[name] = subprocess.Popen(
                [sys.executable, str(Path(__file__).with_name('terminal_exec.py')), *command],
                cwd=ROOT, env=environment, stdin=slave, stdout=slave, stderr=slave,
                start_new_session=True)
        finally:
            os.close(master)
            os.close(slave)

    def can_start(self, component):
        if self.stopping or self.initialize_dir or self.pending_start or 'initialize' in self.processes:
            return False
        if component == 'plc':
            return self.conveyor and self.plc.phase is None
        return component not in self.processes

    def start_isaac(self):
        self._request_start('isaac')

    def start_ros(self):
        self._request_start('ros')

    def start_plc(self):
        self._request_start('plc')

    def _request_start(self, component):
        if not self.can_start(component):
            return
        self.error = ''
        try:
            if self.run is None:
                if not self.lock.tryLock(0):
                    self.error = '다른 ARM Cell Launcher가 실행 중입니다.'
                    self.validate()
                    return
                self._prepare_run(component)
            if not self.pending_start:
                self._launch(component)
        except Exception as exc:
            self.error = str(exc)
            self.begin_stop()
        self.validate()

    def _prepare_run(self, component):
        self.plc.closing = False
        self.plc.message = ''
        self.ros_preflight = None
        self.run = tempfile.TemporaryDirectory(prefix='arm-cell-launcher-')
        entry = self.selection()
        self.runtime = Path(self.run.name) / 'runtime_configuration.json'
        self.environment = dict(
            os.environ, SFTWIN_PROJECT_ROOT=str(ROOT), ARM_CELL_PROFILE=str(self.runtime),
            ARM_CELL_RUNTIME_CONFIG=str(self.runtime), PYTHONUNBUFFERED='1',
            ARM_CELL_INITIALIZE_REQUEST=str(self.runtime.with_name('initialize.request')),
        )
        if read(entry['profile']).get('kind') != 'arm-cell-configured-supply-profile':
            export_selection(entry, self.runtime)
            return

        producer_environment = dict(
            self.environment, PYTHONPATH=str(ROOT / 'ros2_ws/src/bringup') + os.pathsep + os.environ.get('PYTHONPATH', ''),
        )
        process = subprocess.Popen(
            preparation_command(entry, self.runtime), cwd=ROOT, env=producer_environment,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True,
        )
        self.processes['prepare'] = process
        os.set_blocking(process.stdout.fileno(), False)
        self.preparation_output = b''
        self.pending_start = component
        self.preparation_started_at = time.monotonic()

    def _launch(self, component):
        if component == 'isaac':
            self.spawn('isaac', ['bash', str(ROOT/'infra/isaac_sim/scripts/isaac_arm_cell.sh'),
                                '--runtime-config', str(self.runtime)], self.environment)
        elif component == 'ros':
            args = [str(self.runtime), 'debug' if self.debug.isChecked() else 'info']
            if self.conveyor:
                self.ros_preflight = self.runtime.with_name('ros-preflight-complete')
                args.append(str(self.ros_preflight))
            self.spawn('ros', ['bash', str(ROOT / 'infra/launcher/start_ros.bash'), *args], self.environment)
        elif component == 'plc':
            if self.plc_gui.isChecked():
                self.plc.open_editor(self.plc_project, self.environment)
            self.plc.start(self.runtime, self.environment)

    def _poll_preparation(self):
        process = self.processes['prepare']
        while True:
            try:chunk = os.read(process.stdout.fileno(),65536)
            except BlockingIOError:break
            if not chunk:break
            self.preparation_output = (self.preparation_output+chunk)[-65536:]
        code = process.poll()
        if code is None or groups_alive([process]):return
        component = self.pending_start
        self.pending_start = None
        process.stdout.close()
        if code != 0 or not self.runtime.is_file():
            lines = self.preparation_output.decode(errors='replace').strip().splitlines()
            self.error = '설정 준비 실패: '+(lines[-1] if lines else f'프로세스 종료 코드 {code}')
            self.begin_stop()
            return
        del self.processes['prepare']
        try:
            self._launch(component)
        except Exception as exc:
            self.error = str(exc)
            self.begin_stop()

    def choose_plc_project(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, 'CODESYS 프로젝트의 Application 선택', str(self.plc_project.parent), 'CODESYS Application (*.iecapp)')
        if filename:
            self.plc_project = Path(filename)
            self.plc_project_button.setText(f'프로젝트 선택… · {self.plc_project.parent.name}')
            self.plc_project_button.setToolTip(filename)
            self.plc_gui.setChecked(True)

    def signal_groups(self, sig):
        signal_processes(self.processes.values(), sig)

    def begin_stop(self):
        self.plc.hold_service = False
        if self.initialize_dir:
            (self.initialize_dir / 'error').write_text('런처에서 실행을 중지했습니다.')
            self.initialize_dir = self.initialize_phase = None
        self.pending_start = None
        if not self.stopping:
            self.stopping = True
            self.plc.close_session()
            self.phase = 0
            self.terminal_stop = False
            self.signal_groups(signal.SIGINT)
            self.deadline = time.monotonic() + 15
        self.validate()

    def _initialize_failed(self, message):
        self.error = message
        (self.initialize_dir / 'error').write_text(message)
        self.initialize_dir = self.initialize_phase = None
        self.plc.hold_service = False

    def _retry_initialize_plc(self, attempt):
        now = time.monotonic()
        if now >= self.initialize_retry_at:
            self.initialize_retry_at = now + 2.
            self.plc.start(self.runtime, dict(self.environment, ARM_CELL_INITIALIZE_ATTEMPT=str(attempt)))

    def _poll_initialize(self):
        if not self.run or self.stopping or self.pending_start:
            return
        attempt = self.initialize_dir
        if attempt is None:
            request = self.runtime.with_name('initialize.request')
            if not request.is_file():
                return
            name = request.read_text().strip()
            request.unlink()
            if Path(name).name != name or not name.startswith('initialize-'):
                raise ValueError('잘못된 초기화 요청')
            attempt = self.runtime.with_name(name)
            if not attempt.is_dir():
                raise ValueError('초기화 요청 디렉터리가 없습니다.')
            self.initialize_dir = attempt
            self.initialize_phase = 'drain'
            self.error = ''
            self.plc.hold_service = True
            self.plc.close_session()
            self.ros_preflight = None
            self.runtime.with_name('ros-preflight-complete').unlink(missing_ok=True)
            if 'ros' in self.processes:
                signal_processes([self.processes['ros']], signal.SIGINT)
            (attempt / 'stop').touch()
            self.drain_started = time.monotonic()
            self.initialize_retry_at = self.drain_started
        if self.initialize_phase in ('drain', 'restart_ros'):
            elapsed = time.monotonic() - self.drain_started
            for collection in (self.processes, self.terminals):
                ros = collection.get('ros')
                if ros is None:
                    continue
                if groups_alive([ros]):
                    if collection is self.terminals or elapsed > 15:
                        signal_processes([ros], signal.SIGKILL if elapsed > 20 else signal.SIGTERM)
                    return
                del collection['ros']
            if self.initialize_phase == 'restart_ros':
                if time.monotonic() < self.initialize_retry_at:
                    return
                self.initialize_phase = 'scene'
        if (attempt / 'error').exists():
            self._initialize_failed((attempt / 'error').read_text())
            return
        if self.initialize_phase == 'drain':
            if self.plc.phase is not None or not (attempt / 'stopped').exists():
                return
            self.plc.closing = False
            if self.conveyor:
                self.initialize_phase = 'probe'
                self.plc.start(self.runtime, dict(self.environment, ARM_CELL_INITIALIZE_ATTEMPT=str(attempt)))
                return
            self.initialize_phase = 'rebuild'
        if self.initialize_phase == 'probe':
            if self.plc.phase is None:
                self._retry_initialize_plc(attempt)
            elif self.plc.phase == 'awaiting_ros':
                self.processes['initialize'] = subprocess.Popen(
                    plc_control.runtime_command('initialize', self.runtime),
                    env=dict(self.environment, ARM_CELL_INITIALIZE_ATTEMPT=str(attempt)),
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
                self.initialize_phase = 'epoch'
            return
        if self.initialize_phase == 'epoch':
            helper = self.processes['initialize']
            if helper.poll() is None:
                return
            output = helper.stdout.read().decode(errors='replace').strip()
            helper.stdout.close()
            del self.processes['initialize']
            print('[Initialize]', output, flush=True)
            if helper.returncode:
                self._initialize_failed(output or 'PLC 초기화 실패')
                return
            self.initialize_phase = 'rebuild'
        if self.initialize_phase == 'rebuild':
            (attempt / 'rebuild').touch()
            self.initialize_phase = 'scene'
        if self.initialize_phase == 'scene' and (attempt / 'ready').exists():
            self._launch('ros')
            self.initialize_phase = 'ros'
        if self.initialize_phase == 'ros':
            if self.processes['ros'].poll() is not None:
                self.initialize_phase = 'restart_ros'
                self.drain_started = time.monotonic()
                self.initialize_retry_at = self.drain_started + 2.
                if self.ros_preflight is not None:
                    self.ros_preflight.unlink(missing_ok=True)
                signal_processes([self.processes['ros']], signal.SIGTERM)
                return
            if self.conveyor and self.plc.phase is None:
                self._retry_initialize_plc(attempt)
            elif not self.conveyor or (self.plc.phase == 'adapter'
                    and (attempt / 'adapter-ready').is_file()
                    and (attempt / 'adapter-ready').read_text() == (attempt / 'stream').read_text()):
                (attempt / 'complete').touch()
                self.initialize_dir = self.initialize_phase = None
                self.plc.hold_service = False

    def poll(self):
        try:
            self._poll_initialize()
        except (OSError, RuntimeError, ValueError) as error:
            if self.initialize_dir:
                self._initialize_failed(str(error))
            else:
                self.error = str(error)
        if self.pending_start and not self.stopping:
            self._poll_preparation()
        if not self.stopping:
            for name, process in self.processes.items():
                if name in ('prepare', 'initialize') or (name == 'ros' and self.initialize_phase in ('drain', 'restart_ros')):
                    continue  # _poll_preparation handles its exit and starts the requested component.
                code = process.poll()
                if code is not None:
                    self.error = f'{name} 프로세스 종료 (코드 {code}).'
                    self.begin_stop()
                    break
            for name, terminal in self.terminals.items():
                if name == 'ros' and self.initialize_phase in ('drain', 'restart_ros'):
                    continue
                code = terminal.poll()
                if code is not None and not self.stopping:
                    self.error = f'{name} 터미널 종료 (코드 {code}). 실행 환경을 정리합니다.'
                    self.begin_stop()
                    break
        if (not self.stopping and self.plc.phase == 'awaiting_ros' and 'ros' in self.processes
                and self.ros_preflight is not None and self.ros_preflight.exists()):
            self.plc.start_adapter()
        if self.stopping:
            # Keep runtime group IDs after leader exit, including when a user
            # closes its terminal. The snapshot outlives every consumer group.
            if groups_alive(self.processes.values()):
                if time.monotonic() >= self.deadline:
                    self.signal_groups(signal.SIGTERM if self.phase == 0 else signal.SIGKILL)
                    self.phase += 1
                    self.deadline = time.monotonic() + 5
            else:
                # Preserve terminals while runtimes handle graceful shutdown.
                if not self.terminal_stop:
                    signal_processes(self.terminals.values(), signal.SIGTERM)
                    self.terminal_stop = True
                    self.deadline = time.monotonic() + 5
                if groups_alive(self.terminals.values()):
                    if time.monotonic() >= self.deadline:
                        signal_processes(self.terminals.values(), signal.SIGKILL)
                elif self.plc.phase is None:
                    for process in self.processes.values():
                        if process.stdout is not None:process.stdout.close()
                    self.processes.clear()
                    self.terminals.clear()
                    self.stopping = False
                    if self.run is not None:
                        self.run.cleanup()
                        self.run = None
                    self.lock.unlock()
                    self.stopping = False
                    if self.closing:
                        self.close()
        self.validate()

    def closeEvent(self, event):
        if self.run is not None or self.stopping:
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
