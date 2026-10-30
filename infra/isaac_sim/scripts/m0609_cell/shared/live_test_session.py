"""Best-effort Isaac-owned live test session descriptors."""

import fcntl
import json
import os
from pathlib import Path
import subprocess
import uuid
from datetime import datetime, timezone


COMPONENTS = ("vision", "motion", "gripper", "safety", "orchestration")


def _git_provenance(project_root):
    """Describe the source worktree; this is not binary build provenance."""
    result = {
        "source": "git worktree",
        "source_root": str(Path(project_root).resolve()),
        "revision": None,
        "dirty_state": "unavailable",
        "unavailable_reason": None,
    }
    try:
        revision = subprocess.run(
            ["git", "-C", str(project_root), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            timeout=0.1,
            check=False,
        )
        if revision.returncode != 0:
            result["unavailable_reason"] = "revision_command_failed"
            return result
        result["revision"] = revision.stdout.strip()
        status = subprocess.run(
            ["git", "-C", str(project_root), "status", "--porcelain"],
            capture_output=True,
            text=True,
            timeout=0.1,
            check=False,
        )
        if status.returncode != 0:
            result["revision"] = None
            result["unavailable_reason"] = "status_command_failed"
            return result
        result["dirty_state"] = "dirty" if status.stdout.strip() else "clean"
    except FileNotFoundError:
        result["unavailable_reason"] = "git_unavailable"
    except subprocess.TimeoutExpired:
        result["revision"] = None
        result["unavailable_reason"] = "git_query_timeout"
    except OSError:
        result["revision"] = None
        result["unavailable_reason"] = "git_query_failed"
    return result


def _process_start_token(pid):
    stat = Path(f"/proc/{int(pid)}/stat").read_text(encoding="utf-8")
    return stat.rsplit(")", 1)[1].split()[19]


class LiveTestSession:
    """Small file-backed session index; it is evidence, never runtime authority."""

    def __init__(self, artifact_root, detail_selection=()):
        self.session_id = str(uuid.uuid4())
        self.started_at = datetime.now(timezone.utc).isoformat()
        now = datetime.now().astimezone()
        project_root = Path(
            os.environ.get("SFTWIN_PROJECT_ROOT", Path(artifact_root).resolve().parent)
        )
        parent = Path(artifact_root) / "diagnostic_records" / now.strftime("%Y%m%d")
        parent.mkdir(parents=True, exist_ok=True)
        base = now.strftime("%H%M%S")
        candidate = parent / base
        suffix = 1
        while candidate.exists():
            candidate = parent / f"{base}-{suffix:02d}"
            suffix += 1
        candidate.mkdir()
        self.root = candidate
        self.descriptor = candidate / "session.json"
        artifact_root = Path(artifact_root)
        self.data = {
            "schema_version": 1,
            "session_id": self.session_id,
            "status": "open",
            "started_at": self.started_at,
            "ended_at": None,
            "end_condition": None,
            "diagnostic_selection": sorted(detail_selection),
            "participants": [
                {
                    "process": "isaac",
                    "instance_id": f"isaac-{os.getpid()}",
                    "process_start_token": _process_start_token(os.getpid()),
                    "status": "participating",
                    "git_provenance": _git_provenance(project_root),
                }
            ],
            "artifacts": {},
            "recording_locations": {
                **{name: f"{name}/" for name in COMPONENTS},
                "motion_planning_details": str(
                    candidate
                    / "motion"
                    / "motion_planning_records"
                    / "motion_planning_details"
                ),
                "motion_planning_summaries": str(
                    candidate
                    / "motion"
                    / "motion_planning_records"
                    / "motion_planning_summaries"
                ),
                "pick_execution_records": str(
                    candidate / "motion" / "pick_execution_records"
                ),
                "motion_profile_traces": str(
                    candidate / "motion" / "motion_profile_trace"
                ),
            },
            "artifact_locations": {
                "vision_snapshots": {
                    "path": str(
                        artifact_root
                        / "snapshots"
                        / candidate.parent.name
                        / candidate.name
                    ),
                    "status": "unconfirmed",
                },
                "rawpart_pose_recordings": {
                    "path": str(artifact_root / "isaac_pose_recordings"),
                    "status": "unconfirmed",
                },
            },
        }
        for name in COMPONENTS:
            (candidate / name).mkdir()
        self._write()

    def _write(self):
        temporary = self.descriptor.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.data, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        os.replace(temporary, self.descriptor)

    def finish(self, condition="timeline_stop"):
        if self.data["status"] != "open":
            return (
                self.descriptor.resolve() if self.data["status"] == "closed" else None
            )
        try:
            with _descriptor_lock(self.descriptor):
                try:
                    latest = json.loads(self.descriptor.read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    # An unreadable/incomplete descriptor is not evidence of a
                    # successful stop. Leave it untouched for later inspection.
                    return
                if (
                    not isinstance(latest, dict)
                    or latest.get("session_id") != self.session_id
                    or latest.get("status") != "open"
                ):
                    return None
                original = self.data
                latest["status"] = "closed"
                latest["ended_at"] = datetime.now(timezone.utc).isoformat()
                latest["end_condition"] = condition
                for participant in latest.get("participants", []):
                    if participant.get("status") == "participating":
                        participant["status"] = "session_ended"
                        participant["end_condition"] = condition
                self.data = latest
                try:
                    self._write()
                except (OSError, TypeError, ValueError):
                    self.data = original
                    return None
                return self.descriptor.resolve()
        except (OSError, TypeError, ValueError):
            # Preserve best-effort evidence behavior at the Isaac Stop boundary.
            pass
        return None


def find_open_session(artifact_root):
    """Return the sole explicitly open session; fail closed on stale/ambiguous data."""
    root = Path(artifact_root) / "diagnostic_records"
    matches = []
    unreadable = False
    if not root.is_dir():
        return None
    for descriptor in root.glob("[0-9]*/**/session.json"):
        try:
            data = json.loads(descriptor.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                unreadable = True
                continue
        except (OSError, ValueError):
            unreadable = True
            continue
        if data.get("status") == "open" and data.get("session_id"):
            matches.append((descriptor, data))
    return matches[0] if len(matches) == 1 and not unreadable else None


class _descriptor_lock:
    """Serialize the two existing local processes updating one descriptor."""

    def __init__(self, descriptor):
        self.path = Path(str(descriptor) + ".lock")

    def __enter__(self):
        self.stream = self.path.open("a+")
        fcntl.flock(self.stream, fcntl.LOCK_EX)
        return self

    def __exit__(self, *_args):
        fcntl.flock(self.stream, fcntl.LOCK_UN)
        self.stream.close()


def join_session(artifact_root, process, instance_id):
    """Record a participant only when exactly one explicit open session exists."""
    found = find_open_session(artifact_root)
    if found is None:
        return None
    descriptor, data = found
    project_root = Path(artifact_root).resolve().parent
    data["participants"].append(
        {
            "process": process,
            "instance_id": instance_id,
            "status": "participating",
            "git_provenance": _git_provenance(project_root),
        }
    )
    temporary = descriptor.with_suffix(".json.tmp")
    try:
        with _descriptor_lock(descriptor):
            latest = json.loads(descriptor.read_text(encoding="utf-8"))
            if (
                latest.get("status") != "open"
                or latest.get("session_id") != data["session_id"]
            ):
                return None
            participants = latest.setdefault("participants", [])
            new_participant = data["participants"][-1]
            if not any(
                item.get("instance_id") == new_participant["instance_id"]
                for item in participants
            ):
                participants.append(new_participant)
            latest.setdefault("artifacts", {}).update(data.get("artifacts", {}))
            temporary.write_text(
                json.dumps(latest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            os.replace(temporary, descriptor)
    except (OSError, ValueError):
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        return None
    return descriptor, data
