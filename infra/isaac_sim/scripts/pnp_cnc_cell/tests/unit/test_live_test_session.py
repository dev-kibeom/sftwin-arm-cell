import json

import pytest
from datetime import datetime as RealDatetime

import shared.live_test_session as session_module
from shared.live_test_session import LiveTestSession, find_open_session, join_session


def test_session_descriptor_uses_unique_path_and_stop_closes(tmp_path, monkeypatch):
    monkeypatch.delenv("SFTWIN_PROJECT_ROOT", raising=False)
    class _FixedDatetime:
        @classmethod
        def now(cls, tz=None):
            return RealDatetime(2026, 10, 20, 12, 0, 0).astimezone(tz)

    monkeypatch.setattr(session_module, "datetime", _FixedDatetime)
    first = LiveTestSession(tmp_path)
    second = LiveTestSession(tmp_path)
    assert first.root != second.root
    assert first.root == tmp_path / "20261020/120000/diagnostic"
    assert second.root == tmp_path / "20261020/120000-01/diagnostic"
    first_data = json.loads(first.descriptor.read_text())
    assert first_data["status"] == "open"
    isaac_provenance = first_data["participants"][0]["git_provenance"]
    assert isaac_provenance["source"] == "git worktree"
    assert isaac_provenance["source_root"] == str(tmp_path.parent.resolve())
    assert isaac_provenance["dirty_state"] == "unavailable"
    assert isaac_provenance["unavailable_reason"] in {
        "revision_command_failed",
        "git_query_failed",
        "git_query_timeout",
    }
    assert first_data["recording_locations"]["gripper"] == "gripper/"
    assert "vision_snapshots" not in first_data["artifacts"]
    assert "rawpart_pose_recordings" not in first_data["artifacts"]
    assert (
        first_data["artifact_locations"]["vision_snapshots"]["status"] == "unconfirmed"
    )
    assert first_data["artifact_locations"]["vision_snapshots"]["path"] == str(
        tmp_path / "20261020/120000/snapshots"
    )
    for component in ("vision", "motion", "gripper", "safety", "orchestration"):
        assert not (first.root / component).exists()

    first.finish()
    first.finish()
    closed = json.loads(first.descriptor.read_text())
    assert closed["status"] == "closed"
    assert closed["end_condition"] == "timeline_stop"
    assert closed["ended_at"]
    second.finish()


def test_join_requires_one_open_session_and_never_selects_closed_session(tmp_path):
    artifact_root = tmp_path / ".run_records"
    session = LiveTestSession(artifact_root)
    found = find_open_session(artifact_root)
    assert found[0] == session.descriptor
    joined = join_session(artifact_root, "ros_bringup", "ros-bringup-123")
    assert joined[1]["session_id"] == session.session_id
    participants = json.loads(session.descriptor.read_text())["participants"]
    assert participants[-1]["instance_id"] == "ros-bringup-123"
    assert participants[-1]["git_provenance"]["source"] == "git worktree"
    assert participants[-1]["git_provenance"]["dirty_state"] == "unavailable"
    other = LiveTestSession(artifact_root)
    assert find_open_session(artifact_root) is None
    other.finish()
    session.finish()
    assert find_open_session(artifact_root) is None


def test_git_provenance_records_revision_and_worktree_dirty_state(
    tmp_path, monkeypatch
):
    calls = []

    class _Result:
        returncode = 0
        stdout = "abc123\n"

    def _run(command, **_kwargs):
        calls.append(command[-2:])
        result = _Result()
        if "status" in command:
            result.stdout = " M source.cpp\n"
        return result

    monkeypatch.setattr(session_module.subprocess, "run", _run)
    provenance = session_module._git_provenance(tmp_path)
    assert provenance == {
        "source": "git worktree",
        "source_root": str(tmp_path.resolve()),
        "revision": "abc123",
        "dirty_state": "dirty",
        "unavailable_reason": None,
    }
    assert len(calls) == 2


def test_session_write_failure_does_not_raise_during_finish(tmp_path, monkeypatch):
    session = LiveTestSession(tmp_path)
    monkeypatch.setattr(
        session, "_write", lambda: (_ for _ in ()).throw(OSError("disk full"))
    )
    session.finish()
    assert session.data["status"] == "open"
    assert json.loads(session.descriptor.read_text())["status"] == "open"


def test_incomplete_descriptor_is_not_replaced_with_clean_close(tmp_path):
    session = LiveTestSession(tmp_path)
    session.descriptor.write_text("{incomplete", encoding="utf-8")

    session.finish()

    assert session.descriptor.read_text(encoding="utf-8") == "{incomplete"


def test_join_write_failure_leaves_session_usable_and_unassociated(
    tmp_path, monkeypatch
):
    artifact_root = tmp_path / ".run_records"
    session = LiveTestSession(artifact_root)
    monkeypatch.setattr(
        session_module.os,
        "replace",
        lambda *_args: (_ for _ in ()).throw(OSError("read only")),
    )

    assert join_session(artifact_root, "ros_bringup", "ros-bringup-test") is None
    assert json.loads(session.descriptor.read_text())["status"] == "open"


def test_session_search_accepts_legacy_and_rejects_mixed_layout_ambiguity(tmp_path):
    legacy = tmp_path / "diagnostic_records/20261020/120000/session.json"
    legacy.parent.mkdir(parents=True)
    legacy.write_text(json.dumps({"session_id": "legacy", "status": "open"}))
    assert find_open_session(tmp_path)[0] == legacy
    current = LiveTestSession(tmp_path)
    assert find_open_session(tmp_path) is None
    legacy.write_text(json.dumps({"session_id": "legacy", "status": "closed"}))
    assert find_open_session(tmp_path)[0] == current.descriptor


def test_custom_directories_are_used_for_creation_and_discovery(tmp_path, monkeypatch):
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config/run_records.yaml').write_text(
        'root: output/records\ndirectories:\n  diagnostic: logs/diag\n  snapshots: images/captures\n')
    monkeypatch.setenv('SFTWIN_PROJECT_ROOT', str(tmp_path))
    from shared.run_records import run_records_root
    root = run_records_root(tmp_path)
    session = LiveTestSession(root)
    assert session.root.parts[-2:] == ('logs', 'diag')
    assert find_open_session(root)[0] == session.descriptor
    run_root = session.root.parent.parent
    assert session.data['artifact_locations']['vision_snapshots']['path'] == str(
        run_root / 'images/captures')


def test_initial_write_failure_removes_nested_run(tmp_path, monkeypatch):
    import pytest
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config/run_records.yaml').write_text('directories:\n  diagnostic: logs/diag\n')
    monkeypatch.setenv('SFTWIN_PROJECT_ROOT', str(tmp_path))
    monkeypatch.setattr(session_module.os, 'replace', lambda *_: (_ for _ in ()).throw(OSError('full')))
    with pytest.raises(OSError):
        LiveTestSession(tmp_path / '.run_records')
    assert not list((tmp_path / '.run_records').glob('[0-9]*/*'))
    assert not list((tmp_path / '.run_records').rglob('*.tmp'))


@pytest.fixture
def initialization_run(tmp_path, monkeypatch):
    from shared import run_records

    class InitializationTime:
        @classmethod
        def now(cls):
            return RealDatetime(2026, 11, 11, 16, 4, 9).astimezone()

    class PlayTime:
        @classmethod
        def now(cls):
            return RealDatetime(2026, 11, 11, 16, 4, 10).astimezone()

    monkeypatch.setenv("SFTWIN_PROJECT_ROOT", str(tmp_path))
    monkeypatch.setattr(run_records, "datetime", InitializationTime)
    monkeypatch.setattr(session_module, "datetime", PlayTime)
    simulation = run_records.diagnostic_component_directory(tmp_path, "simulation")
    simulation.mkdir()
    (simulation / "initialization.json").write_text("initialization evidence")
    return tmp_path / ".run_records", simulation.parent


def test_first_play_adopts_initialization_run_and_preserves_evidence(initialization_run):
    artifact_root, diagnostic = initialization_run
    session = LiveTestSession(artifact_root, ("vision",))

    assert session.root == diagnostic
    assert session.root == artifact_root / "20261111/160409/diagnostic"
    assert (diagnostic / "simulation/initialization.json").read_text() == "initialization evidence"
    data = json.loads(session.descriptor.read_text())
    assert data["session_id"] == session.session_id
    assert data["session_association"] == "associated"
    assert data["status"] == "open"
    assert data["owner"] == "isaac"
    assert data["diagnostic_selection"] == ["vision"]
    assert RealDatetime.fromisoformat(data["started_at"]).strftime("%H%M%S") == "160410"
    assert data["artifact_locations"]["vision_snapshots"]["path"] == str(
        artifact_root / "20261111/160409/snapshots"
    )
    assert len(list(artifact_root.glob("[0-9]*/*"))) == 1
    assert find_open_session(artifact_root)[0] == session.descriptor
    assert join_session(artifact_root, "ros_bringup", "ros-test")[1]["session_id"] == session.session_id


def test_second_play_creates_new_run_after_initialization_handoff(initialization_run):
    artifact_root, diagnostic = initialization_run
    first = LiveTestSession(artifact_root)
    first.finish()
    closed_descriptor = first.descriptor.read_text()
    second = LiveTestSession(artifact_root)

    assert first.root == diagnostic
    assert second.root == artifact_root / "20261111/160410/diagnostic"
    assert first.session_id != second.session_id
    assert first.descriptor.read_text() == closed_descriptor
    assert json.loads(closed_descriptor)["status"] == "closed"
    assert find_open_session(artifact_root)[0] == second.descriptor


def test_promotion_releases_standalone_cache_for_later_initialization(initialization_run):
    from shared.run_records import diagnostic_component_directory

    artifact_root, diagnostic = initialization_run
    session = LiveTestSession(artifact_root)
    session.finish()
    next_initialization = diagnostic_component_directory(artifact_root.parent, "simulation")

    assert session.root == diagnostic
    assert next_initialization.parent != diagnostic
    assert json.loads((next_initialization.parent / "session.json").read_text())["session_id"] is None


@pytest.mark.parametrize("metadata", [
    {"session_id": None, "session_association": "unassociated", "owner_pid": -1},
    {"session_id": "already-associated", "session_association": "unassociated"},
    {"session_id": None, "session_association": "associated"},
    {"session_id": None, "session_association": "unassociated", "status": "closed"},
    {},
    [],
    "malformed",
    None,
])
def test_ineligible_initialization_descriptor_is_preserved(initialization_run, metadata):
    artifact_root, diagnostic = initialization_run
    descriptor = diagnostic / "session.json"
    if metadata is None:
        descriptor.unlink()
    elif metadata == "malformed":
        descriptor.write_text("{incomplete")
    else:
        if isinstance(metadata, dict) and metadata:
            metadata.setdefault("owner_pid", session_module.os.getpid())
        descriptor.write_text(json.dumps(metadata))
    original = descriptor.read_text() if descriptor.exists() else None

    session = LiveTestSession(artifact_root)

    assert session.root != diagnostic
    assert (descriptor.read_text() if descriptor.exists() else None) == original
    assert (diagnostic / "simulation/initialization.json").read_text() == "initialization evidence"


def test_failed_promotion_preserves_initialization_for_retry(initialization_run, monkeypatch):
    artifact_root, diagnostic = initialization_run
    descriptor = diagnostic / "session.json"
    original = descriptor.read_text()
    replace = session_module.os.replace

    def fail_replace(*_args):
        raise OSError("disk full")

    monkeypatch.setattr(session_module.os, "replace", fail_replace)
    with pytest.raises(OSError, match="disk full"):
        LiveTestSession(artifact_root)
    assert descriptor.read_text() == original
    assert (diagnostic / "simulation/initialization.json").read_text() == "initialization evidence"
    assert not list(artifact_root.rglob("*.tmp"))
    monkeypatch.setattr(session_module.os, "replace", replace)

    session = LiveTestSession(artifact_root)
    assert session.root == diagnostic
    assert len(list(artifact_root.glob("[0-9]*/*"))) == 1


def test_initialization_handoff_honors_nested_record_directories(tmp_path, monkeypatch):
    from shared.run_records import diagnostic_component_directory, run_records_root

    (tmp_path / "config").mkdir()
    (tmp_path / "config/run_records.yaml").write_text(
        "root: output/records\ndirectories:\n  diagnostic: logs/diag\n  snapshots: images/captures\n"
    )
    monkeypatch.setenv("SFTWIN_PROJECT_ROOT", str(tmp_path))
    diagnostic = diagnostic_component_directory(tmp_path, "simulation").parent
    artifact_root = run_records_root(tmp_path)

    session = LiveTestSession(artifact_root)

    assert session.root == diagnostic
    assert session.data["artifact_locations"]["vision_snapshots"]["path"] == str(
        diagnostic.parent.parent / "images/captures"
    )
    assert find_open_session(artifact_root)[0] == session.descriptor


def test_first_play_adopts_initialization_run_after_helper_reload(initialization_run):
    import importlib
    from shared import run_records

    artifact_root, diagnostic = initialization_run
    # snapshot_camera.py reloads the shared helpers after setup_scene.py.
    importlib.reload(run_records)

    session = LiveTestSession(artifact_root)

    assert session.root == diagnostic
    assert len(list(artifact_root.glob("[0-9]*/*"))) == 1
