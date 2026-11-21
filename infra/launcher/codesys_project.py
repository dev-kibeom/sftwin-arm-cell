"""Prepare a CODESYS 4 project, then replace this process with its IDE session."""
import json
import os
import socket
from pathlib import Path
import subprocess
import sys
import tempfile


DEFAULT_PROJECT = Path(__file__).resolve().parents[1] / 'plc/conveyor_pnp_cell/Conveyor.fbsdev'


def prepare_communication(project):
    settings = project / 'Communication.settings.json'
    gateway = {'type': 'CODESYS.GwV3', 'name': '', 'hostName': 'localhost', 'port': 1217}
    try:
        previous = settings.read_bytes()
    except FileNotFoundError:
        previous = None

    try:
        config = json.loads(previous) if previous is not None else {}
    except ValueError:
        return
    if not isinstance(config, dict) or config.get('gateway') not in (None, gateway):
        return

    device = config.get('device')
    if device is None:
        device = {}
    if not isinstance(device, dict):
        return
    if device.get('nodeName') and config.get('gateway') is not None:
        return

    if not device.get('nodeName'):
        device['nodeName'] = os.environ.get('ARM_CELL_CODESYS_NODE_NAME', '').strip() or socket.gethostname()
    config.update(device=device, gateway=gateway)
    seed = tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=project, delete=False)
    try:
        with seed:
            json.dump(config, seed, indent=2)
            seed.write('\n')
        if previous is None:
            try:
                os.link(seed.name, settings)
            except FileExistsError:
                pass
        elif settings.read_bytes() == previous:
            os.replace(seed.name, settings)
    finally:
        Path(seed.name).unlink(missing_ok=True)


def prepare(selected):
    selected = selected.expanduser().resolve()
    if selected.suffix == '.iecapp':
        project = selected.parent
        applications = [selected]
    else:
        project = selected
        applications = sorted(project.glob('*.iecapp'))
    if project.suffix != '.fbsdev' or not applications:
        raise ValueError(f'Application 파일(*.iecapp)을 찾을 수 없습니다 · {project}')
    for application in applications:
        if not application.is_file() or not application.with_suffix('.iecapp^').is_dir():
            raise ValueError(f'CODESYS Application 소스 확인 필요 · {application}')

    prepare_communication(project)
    for application in applications:
        lock = application.with_suffix('.iecapp^') / 'Libraries.lock.json'
        if lock.is_file():
            subprocess.run(['c4-cli', 'library', 'restore', str(lock)], check=True)
    return project


if __name__ == '__main__':
    try:
        project = prepare(Path(sys.argv[1]))
        os.execlp('c4-cli', 'c4-cli', 'standalone-session', str(project))
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        sys.exit(f'CODESYS 프로젝트 준비 실패 · {exc}')
