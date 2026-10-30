"""Bounded local pose history for Isaac-generated RawParts."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import uuid
from typing import Any


SAMPLE_INTERVAL_SECONDS = 0.2
POSITION_THRESHOLD_M = 0.001
ROTATION_THRESHOLD_DEG = 1.0
MAX_SESSION_BYTES = 8 * 1024 * 1024
MAX_SESSIONS = 20


class RawPartPoseRecorder:
    """Record changed PhysX poses from the Material Supply Kit update path."""

    def __init__(
        self,
        stage: Any,
        artifact_directory: Path,
        live_test_session_id: str | None = None,
    ):
        self.stage = stage
        self.artifact_directory = Path(artifact_directory)
        self.live_test_session_id = live_test_session_id
        self.session_id: str | None = None
        self.session_path: Path | None = None
        self._last_samples: dict[str, dict[str, Any]] = {}
        self._last_sampled_at = -math.inf
        self._last_sim_time_ns: int | None = None
        self._simulation_epoch = 0
        self._disabled_at_capacity = False

    def sample(self, physx: Any, simulation_time_seconds: float, *, now: float) -> None:
        if now - self._last_sampled_at < SAMPLE_INTERVAL_SECONDS:
            return
        self._last_sampled_at = now
        if self._disabled_at_capacity:
            return
        current: dict[str, dict[str, Any]] = {}
        complete = True
        for prim in self.stage.Traverse():
            if not prim.IsValid() or not prim.GetName().startswith("RawPart_"):
                continue
            path = str(prim.GetPath())
            if "/MaterialSupply/Deliveries/" not in path:
                continue
            state = physx.get_rigidbody_transformation(path)
            if not state or not state.get("ret_val"):
                complete = False
                break
            position = tuple(float(value) for value in state["position"])
            orientation = tuple(float(value) for value in state["rotation"])
            if (
                len(position) != 3
                or len(orientation) != 4
                or not all(math.isfinite(value) for value in (*position, *orientation))
            ):
                complete = False
                break
            current[path] = {
                "rawpart_id": self._identity(prim, path),
                "prim_path": path,
                "position_xyz": list(position),
                "orientation_xyzw": list(orientation),
            }
            dimensions = self._attribute_value(prim, "sf_twin:dimensions_m")
            mass = self._attribute_value(prim, "physics:mass")
            if dimensions is None or mass is None:
                complete = False
                break
            dimensions = tuple(float(value) for value in dimensions)
            mass = float(mass)
            if (
                len(dimensions) != 3
                or not all(math.isfinite(value) and value > 0 for value in dimensions)
                or not math.isfinite(mass)
                or mass <= 0
            ):
                complete = False
                break
            current[path]["dimensions_m"] = list(dimensions)
            current[path]["mass_kg"] = mass
        if not complete:
            return

        sim_time_ns = max(0, round(float(simulation_time_seconds) * 1_000_000_000))
        if self._last_sim_time_ns is not None and sim_time_ns < self._last_sim_time_ns:
            self.stop()
            self._simulation_epoch += 1
            self._last_sampled_at = now
        self._last_sim_time_ns = sim_time_ns
        if self.session_id is None:
            self._start_session(sim_time_ns)
        for path, pose in current.items():
            previous = self._last_samples.get(path)
            if previous is None:
                self._append("created", pose, sim_time_ns)
            elif _pose_changed(previous, pose):
                self._append("pose_changed", pose, sim_time_ns)
        for path, previous in self._last_samples.items():
            if path not in current:
                self._append("removed", previous, sim_time_ns)
        # Thresholds compare to the last persisted pose, so sub-threshold motion
        # accumulates and is not lost when it eventually crosses the threshold.
        updated = dict(self._last_samples)
        for path, pose in current.items():
            if path not in updated or _pose_changed(updated[path], pose):
                updated[path] = pose
        for path in set(updated).difference(current):
            updated.pop(path, None)
        self._last_samples = updated

    def stop(self) -> None:
        self.session_id = None
        self.session_path = None
        self._last_samples.clear()
        self._last_sampled_at = -math.inf
        self._last_sim_time_ns = None
        self._disabled_at_capacity = False

    def bind_live_test_session(self, session_id: str | None) -> None:
        if session_id != self.live_test_session_id:
            self._simulation_epoch = 0
        self.live_test_session_id = session_id

    def _start_session(self, simulation_time_ns: int) -> None:
        self.artifact_directory.mkdir(parents=True, exist_ok=True)
        self.session_id = uuid.uuid4().hex
        self.session_path = self.artifact_directory / f"{self.session_id}.jsonl"
        self._append(
            "session_started",
            {
                "schema_version": 1,
                "session_id": self.session_id,
                "created_at_utc": datetime.now(timezone.utc).isoformat(),
            },
            simulation_time_ns,
        )
        sessions = sorted(
            self.artifact_directory.glob("*.jsonl"),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
        for stale in sessions[MAX_SESSIONS:]:
            stale.unlink(missing_ok=True)

    def _append(self, event: str, pose: dict[str, Any], sim_time_ns: int) -> None:
        assert self.session_path is not None and self.session_id is not None
        record = {
            "session_id": self.session_id,
            "live_test_session_id": self.live_test_session_id,
            "simulation_epoch": self._simulation_epoch,
            "frame_id": "world",
            "observation_point": "physx_rigidbody_pose",
            "simulation_time_basis": "isaac_timeline",
            "simulation_time_ns": sim_time_ns,
            "event": event,
            **pose,
        }
        encoded = json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n"
        size = self.session_path.stat().st_size if self.session_path.exists() else 0
        if size + len(encoded.encode("utf-8")) > MAX_SESSION_BYTES:
            marker = (
                json.dumps(
                    {
                        "session_id": self.session_id,
                        "simulation_time_ns": sim_time_ns,
                        "event": "capture_limit_reached",
                    },
                    separators=(",", ":"),
                    sort_keys=True,
                )
                + "\n"
            )
            if size + len(marker.encode("utf-8")) <= MAX_SESSION_BYTES:
                with self.session_path.open("a", encoding="utf-8") as output:
                    output.write(marker)
            self._disabled_at_capacity = True
            return
        with self.session_path.open("a", encoding="utf-8") as output:
            output.write(encoded)

    @staticmethod
    def _identity(prim: Any, path: str) -> str:
        try:
            delivery = prim.GetAttribute("sf_twin:delivery_id")
            sequence = prim.GetAttribute("sf_twin:delivery_sequence")
            if delivery and sequence:
                return f"{delivery.Get()}:{sequence.Get()}"
        except (AttributeError, RuntimeError):
            pass
        return path

    @staticmethod
    def _attribute_value(prim: Any, name: str) -> Any | None:
        try:
            attribute = prim.GetAttribute(name)
            if attribute:
                value = attribute.Get()
                return tuple(value) if name.endswith("dimensions_m") else value
        except (AttributeError, RuntimeError, TypeError):
            pass
        return None


def _pose_changed(previous: dict[str, Any], current: dict[str, Any]) -> bool:
    old_position = previous["position_xyz"]
    new_position = current["position_xyz"]
    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(old_position, new_position)))
    if distance >= POSITION_THRESHOLD_M:
        return True
    old_q = previous["orientation_xyzw"]
    new_q = current["orientation_xyzw"]
    old_norm = math.sqrt(sum(value * value for value in old_q))
    new_norm = math.sqrt(sum(value * value for value in new_q))
    if old_norm == 0.0 or new_norm == 0.0:
        return True
    dot = abs(sum(a * b for a, b in zip(old_q, new_q)) / (old_norm * new_norm))
    angle = math.degrees(2.0 * math.acos(min(1.0, dot)))
    return angle >= ROTATION_THRESHOLD_DEG
