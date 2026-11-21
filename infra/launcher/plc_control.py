"""Asynchronous host lifecycle for the Conveyor PLC button."""
import ipaddress
import json
import os
from pathlib import Path
import signal

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, QTimer


UNIT = 'codesyscontrol.service'
HERE = Path(__file__).resolve().parent


def runtime_command(stage, runtime):
    if stage == 'probe':
        return ['/usr/bin/python3', str(HERE / 'conveyor_plc.py'), stage, str(runtime)]
    return ['bash', str(HERE / 'start_plc_adapter.bash'), str(runtime)]


class ConveyorPLC(QObject):
    def __init__(self, parent):
        super().__init__(parent)
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.finished.connect(self.finished)
        self.process.errorOccurred.connect(self.process_error)
        self.kill_timer = QTimer(self)
        self.kill_timer.setSingleShot(True)
        self.kill_timer.timeout.connect(self.process.kill)
        self.phase = None
        self.message = ''
        self.owned_invocation = ''
        self.closing = False
        self.runtime = None
        self.output = b''

    def log(self, message):
        self.message = message
        print('[PLC]', message, flush=True)

    def run(self, phase, command):
        self.phase = phase
        self.output = b''
        self.process.start(command[0], command[1:])

    def inspect(self, phase):
        self.run(phase, ['systemctl', 'show', '--property=ActiveState,InvocationID,LoadState', UNIT])

    def start(self, runtime, environment):
        if self.phase or self.closing:
            return
        self.runtime = runtime
        env = QProcessEnvironment()
        for key, value in environment.items():
            env.insert(key, value)
        self.process.setProcessEnvironment(env)
        try:
            config = json.loads(runtime.read_text())['configuration']['cell']['conveyor']
            host = config['connection']['host']
        except (OSError, KeyError, ValueError) as error:
            self.log(f'PLC 실행 설정 확인 실패 · {error}')
            return
        try:
            local = ipaddress.ip_address(host).is_loopback
        except ValueError:
            local = host.lower() == 'localhost'
        if local:
            self.log('CODESYS 서비스 확인 중')
            self.inspect('inspect')
        else:
            self.probe()

    def probe(self):
        self.log('Conveyor PLC 응답 확인 중')
        self.run('probe', runtime_command('probe', self.runtime))

    def read_output(self):
        chunk = bytes(self.process.readAllStandardOutput())
        if chunk:
            self.output = (self.output + chunk)[-8192:]
            if self.phase in ('probe', 'adapter'):
                print(chunk.decode(errors='replace'), end='', flush=True)

    def process_error(self, error):
        if error == QProcess.FailedToStart:
            self.kill_timer.stop()
            if self.phase == 'release':
                self.owned_invocation = ''
            self.failed(self.process.errorString())

    def failed(self, reason):
        self.log(f'{self.message} · 실패: {reason}')
        self.phase = None
        if self.closing:
            self.release()

    def finished(self, code, exit_status):
        self.kill_timer.stop()
        self.read_output()
        phase, self.phase = self.phase, None
        output = self.output.decode(errors='replace').strip()
        if self.closing and phase in ('adapter', 'probe'):
            self.log('PLC 어댑터 종료됨' if phase == 'adapter' else 'PLC 응답 확인 중단됨')
            self.release()
            return
        if code or exit_status == QProcess.CrashExit:
            if phase == 'release':
                self.owned_invocation = ''
            self.failed(output.splitlines()[-1] if output else f'종료 코드 {code}')
            return
        if phase == 'start':
            # Identify our service even when the GUI closes during startup.
            self.inspect('identify')
        elif phase in ('inspect', 'identify', 'release'):
            self.service_observed(phase, output)
        elif phase == 'stop':
            self.log('CODESYS 종료됨')
        elif phase == 'probe':
            self.log('Conveyor PLC 응답 확인 · PLC 어댑터 기동')
            self.run('adapter', runtime_command('adapter', self.runtime))
        elif phase == 'adapter':
            self.log('PLC 어댑터 종료됨')

    def service_observed(self, phase, output):
        fields = dict(line.split('=', 1) for line in output.splitlines() if '=' in line)
        active = fields.get('ActiveState')
        invocation = fields.get('InvocationID', '')
        if phase == 'release':
            if active == 'active' and invocation == self.owned_invocation:
                self.log('런처가 시작한 CODESYS 종료 중')
                self.run('stop', ['systemctl', 'stop', '--no-ask-password', UNIT])
            self.owned_invocation = ''
            return
        if phase == 'identify':
            if active != 'active' or not invocation:
                self.failed(output or '시작한 CODESYS 서비스 식별 불가')
                return
            self.owned_invocation = invocation
        if self.closing:
            self.log('PLC 시작 중단됨')
            self.release()
        elif active == 'active':
            self.probe()
        elif active in ('inactive', 'failed') and fields.get('LoadState') == 'loaded':
            self.log('CODESYS 서비스 시작 중')
            self.run('start', ['systemctl', 'start', '--no-ask-password', UNIT])
        else:
            self.failed(output or 'CODESYS 서비스 상태 확인 불가')

    def close_session(self):
        self.closing = True
        if self.phase in ('probe', 'adapter'):
            pid = self.process.processId()
            if pid:
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            self.kill_timer.start(5000)
        elif self.phase is None:
            self.release()

    def release(self):
        if self.owned_invocation:
            self.inspect('release')
