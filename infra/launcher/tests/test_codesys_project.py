from pathlib import Path
import json
import os
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import codesys_project


def test_prepare_login_and_restore_selected_application(tmp_path, monkeypatch):
    project = tmp_path / 'Line A.fbsdev'
    project.mkdir()
    application = project / 'Feeder.iecapp'
    application.write_text('{}')
    (project / 'Feeder.iecapp^').mkdir()
    lock = application.with_suffix('.iecapp^') / 'Libraries.lock.json'
    lock.write_text('{}')
    calls = []
    monkeypatch.setattr(subprocess, 'run', lambda args, check: calls.append(args))
    assert codesys_project.prepare(application) == project
    settings = project / 'Communication.settings.json'
    initial = json.loads(settings.read_text())
    assert initial['gateway']['hostName'] == 'localhost' and initial['gateway']['port'] == 1217
    custom = '{"gateway":{"hostName":"operator-selected"},"device":{"nodeName":"Line A"}}'
    settings.write_text(custom)
    assert codesys_project.prepare(project) == project
    assert settings.read_text() == custom
    assert calls == [['c4-cli', 'library', 'restore', str(lock)]] * 2


def test_reject_stale_project_directory(tmp_path):
    project = tmp_path / 'Old.fbsdev'
    (project / 'Application.iecapp^' / '.intermediate').mkdir(parents=True)
    with pytest.raises(ValueError, match='iecapp'):
        codesys_project.prepare(project)
    assert not (project / 'Communication.settings.json').exists()


@pytest.mark.parametrize('restore_code', [0, 1])
def test_editor_starts_only_after_restore(tmp_path, restore_code):
    project = tmp_path / 'Example.fbsdev'
    sources = project / 'Application.iecapp^'
    sources.mkdir(parents=True)
    (project / 'Application.iecapp').write_text('{}')
    lock = sources / 'Libraries.lock.json'
    lock.write_text('{}')
    command = tmp_path / 'c4-cli'
    command.write_text('#!/bin/sh\nprintf "%s\\n" "$@" >> "$C4_CALLS"\n'
                       'if [ "$1" = library ]; then exit "$RESTORE_CODE"; fi\n')
    command.chmod(0o755)
    calls = tmp_path / 'calls'
    env = dict(os.environ, PATH=str(tmp_path) + os.pathsep + os.environ['PATH'],
               C4_CALLS=str(calls), RESTORE_CODE=str(restore_code))
    result = subprocess.run([sys.executable, codesys_project.__file__, str(project)], env=env)
    expected = ['library', 'restore', str(lock)]
    if restore_code == 0:
        expected.extend(['standalone-session', str(project)])
    assert result.returncode == restore_code
    assert calls.read_text().splitlines() == expected
