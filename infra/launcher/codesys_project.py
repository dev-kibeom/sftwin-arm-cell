"""Prepare a CODESYS 4 project, then replace this process with its IDE session."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


DEFAULT_PROJECT = Path(__file__).resolve().parents[1] / 'plc/conveyor_pnp_cell/Conveyor.fbsdev'


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

    settings = project / 'Communication.settings.json'
    if not settings.exists():
        # Publish a complete file without replacing another launcher or the IDE's settings.
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=project) as seed:
            json.dump({'device': {'nodeName': None}, 'gateway': {
                'type': 'CODESYS.GwV3', 'name': '', 'hostName': 'localhost', 'port': 1217,
            }}, seed, indent=2)
            seed.flush()
            try:
                os.link(seed.name, settings)
            except FileExistsError:
                pass

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
