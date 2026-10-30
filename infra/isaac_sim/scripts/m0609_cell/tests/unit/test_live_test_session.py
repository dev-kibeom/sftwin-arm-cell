import json
from datetime import datetime as RealDatetime, timezone

import shared.live_test_session as session_module
from shared.live_test_session import LiveTestSession, find_open_session, join_session


def test_session_descriptor_uses_unique_path_and_stop_closes(tmp_path, monkeypatch):
    class _FixedDatetime:
        @classmethod
        def now(cls, tz=None):
            return RealDatetime(2026, 10, 20, 12, 0, 0, tzinfo=tz or timezone.utc)

    monkeypatch.setattr(session_module, "datetime", _FixedDatetime)
    first = LiveTestSession(tmp_path)
    second = LiveTestSession(tmp_path)
    assert first.root != second.root
    assert first.root.name.split("-")[0] == second.root.name.split("-")[0]
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
        tmp_path / "snapshots" / first.root.parent.name / first.root.name
    )
    for component in ("vision", "motion", "gripper", "safety", "orchestration"):
        assert (first.root / component).is_dir()

    first.finish()
    first.finish()
    closed = json.loads(first.descriptor.read_text())
    assert closed["status"] == "closed"
    assert closed["end_condition"] == "timeline_stop"
    assert closed["ended_at"]
    second.finish()


def test_join_requires_one_open_session_and_never_selects_closed_session(tmp_path):
    artifact_root = tmp_path / ".local_artifacts"
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
    artifact_root = tmp_path / ".local_artifacts"
    session = LiveTestSession(artifact_root)
    monkeypatch.setattr(
        session_module.os,
        "replace",
        lambda *_args: (_ for _ in ()).throw(OSError("read only")),
    )

    assert join_session(artifact_root, "ros_bringup", "ros-bringup-test") is None
    assert json.loads(session.descriptor.read_text())["status"] == "open"
