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
