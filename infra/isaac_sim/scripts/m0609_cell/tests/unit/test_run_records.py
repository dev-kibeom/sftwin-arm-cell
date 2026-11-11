"""Run namespaces are stable within a process and distinct between runs."""
import json
from datetime import datetime

from shared import run_records


def test_standalone_records_share_run_and_describe_association(tmp_path):
    vision = run_records.diagnostic_component_directory(tmp_path, 'vision')
    motion = run_records.diagnostic_component_directory(tmp_path, 'motion')
    assert vision.parent == motion.parent
    assert vision.parent.name == 'diagnostic'
    run = vision.parent.parent
    datetime.strptime(run.parent.name, '%Y%m%d')
    datetime.strptime(run.name, '%H%M%S')
    assert run.parent.parent == tmp_path / '.run_records'
    metadata = json.loads((vision.parent / 'session.json').read_text())
    assert metadata['session_association'] == 'unassociated'
    assert metadata['session_id'] is None


def test_separate_process_namespace_collision_gets_suffix(tmp_path, monkeypatch):
    class FixedDatetime:
        @classmethod
        def now(cls):
            return datetime(2026, 11, 11, 15, 30).astimezone()
    monkeypatch.setattr(run_records, 'datetime', FixedDatetime)
    monkeypatch.setattr(run_records.os, 'getpid', lambda: 111)
    first = run_records.standalone_run_directory(tmp_path)
    monkeypatch.setattr(run_records.os, 'getpid', lambda: 222)
    second = run_records.standalone_run_directory(tmp_path)
    assert first == tmp_path / '20261111/153000'
    assert second == tmp_path / '20261111/153000-01'


def test_metadata_failure_removes_run_and_invalidates_cache(tmp_path, monkeypatch):
    import pytest
    original = run_records.os.replace
    monkeypatch.setattr(run_records.os, 'replace', lambda *_: (_ for _ in ()).throw(OSError('full')))
    with pytest.raises(OSError):
        run_records.diagnostic_component_directory(tmp_path, 'vision')
    assert not list((tmp_path / '.run_records').glob('[0-9]*/*'))
    assert (run_records.os.getpid(), (tmp_path / '.run_records').resolve()) not in run_records._RUN_DIRECTORIES
    monkeypatch.setattr(run_records.os, 'replace', original)
    path = run_records.diagnostic_component_directory(tmp_path, 'vision')
    assert (path.parent / 'session.json').is_file()


def test_cleanup_preserves_existing_records(tmp_path):
    run = tmp_path / 'run'
    (run / 'logs/diag/empty').mkdir(parents=True)
    record = run / 'logs/diag/existing.json'
    record.write_text('evidence')
    run_records.remove_empty_run(run)
    assert record.read_text() == 'evidence'
    assert not (run / 'logs/diag/empty').exists()
