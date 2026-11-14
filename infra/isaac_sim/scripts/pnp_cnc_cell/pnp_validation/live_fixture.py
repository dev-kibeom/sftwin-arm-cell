"""Validation-private file protocol and AMR top-plane provenance helpers.

This module deliberately has no Isaac or ROS imports.  The Isaac update loop
and the ROS-side helper use this small, versioned file seam at their boundary.
"""

from __future__ import annotations

import hashlib
import json
import os
import random
import tempfile
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping


SCHEMA_VERSION = 1


def cube_geometry_from_transform(
    matrix: list[list[float]] | tuple[tuple[float, ...], ...], size: float = 1.0
) -> dict[str, object]:
    """Transform a unit Cube's local corners exactly once into world geometry."""
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("cube transform must be a 4x4 matrix")
    if not isinstance(size, (int, float)) or float(size) <= 0.0:
        raise ValueError("cube size must be positive")

    def transform(point):
        return tuple(
            sum(float(matrix[row][column]) * point[column] for column in range(3))
            + float(matrix[row][3])
            for row in range(3)
        )

    half = float(size) / 2.0
    corners = [
        transform((sx * half, sy * half, sz * half, 1.0))
        for sx in (-1.0, 1.0)
        for sy in (-1.0, 1.0)
        for sz in (-1.0, 1.0)
    ]
    minimum = [min(corner[index] for corner in corners) for index in range(3)]
    maximum = [max(corner[index] for corner in corners) for index in range(3)]
    center = transform((0.0, 0.0, 0.0, 1.0))
    return {
        "center": list(center),
        "dimensions_m": [maximum[index] - minimum[index] for index in range(3)],
        "aabb_min": minimum,
        "aabb_max": maximum,
        "top_z_m": maximum[2],
    }


def _atomic_write(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, sort_keys=True, separators=(",", ":"))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _read(path: Path) -> dict[str, object]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError("handoff payload must be an object")
    return value


@contextmanager
def _exclusive_file_lock(path: Path):
    import fcntl

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+", encoding="utf-8") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


class FixtureCommandWriter:
    """The sole writer for a validation run's command file."""

    def __init__(self, path: str | Path, *, run_id: str, fixture_id: str):
        self.path = Path(path)
        self.run_id = run_id
        self.fixture_id = fixture_id
        self._sequence_path = self.path.with_suffix(".seq")

    def write(self, operation: str, payload: Mapping[str, object]) -> dict[str, object]:
        with _exclusive_file_lock(self.path.with_suffix(".lock")):
            try:
                current = int(_read(self._sequence_path)["next_command_id"])
            except FileNotFoundError:
                current = 0
            command_id = current + 1
            command = {
                "schema_version": SCHEMA_VERSION,
                "generation": command_id,
                "command_id": command_id,
                "run_id": self.run_id,
                "fixture_id": self.fixture_id,
                "operation": operation,
                "payload": dict(payload),
            }
            _atomic_write(self._sequence_path, {"next_command_id": command_id})
            _atomic_write(self.path, command)
            return command


class FixtureResponseReader:
    def __init__(self, path: str | Path):
        self.path = Path(path)

    def read_for(self, command: Mapping[str, object]) -> dict[str, object]:
        response = _read(self.path)
        for field in ("command_id", "generation", "run_id", "fixture_id"):
            if response.get(field) != command.get(field):
                raise ValueError(f"response {field} does not match command")
        return response


class FixtureRuntimeConsumer:
    """Idempotent command consumer owned by the Isaac update loop."""

    def __init__(
        self,
        command_path: str | Path,
        response_path: str | Path,
        *,
        run_id: str,
        fixture_id: str,
        handlers: Mapping[str, Callable[[Mapping[str, object]], Mapping[str, object]]],
    ):
        self.command_path = Path(command_path)
        self.response_path = Path(response_path)
        self.run_id = run_id
        self.fixture_id = fixture_id
        self.handlers = handlers
        self._last_command_id = 0
        self._last_response: dict[str, object] | None = None
        if self.response_path.exists():
            try:
                response = _read(self.response_path)
            except (OSError, ValueError, json.JSONDecodeError):
                response = None
            if (
                response
                and response.get("run_id") == self.run_id
                and response.get("fixture_id") == self.fixture_id
                and isinstance(response.get("command_id"), int)
                and isinstance(response.get("generation"), int)
            ):
                self._last_command_id = int(response["command_id"])
                self._last_response = response

    def _response(self, command: Mapping[str, object], status: str, **extra: object):
        response = {
            "schema_version": SCHEMA_VERSION,
            "generation": command.get("generation"),
            "command_id": command.get("command_id"),
            "run_id": command.get("run_id"),
            "fixture_id": command.get("fixture_id"),
            "status": status,
            **extra,
        }
        _atomic_write(self.response_path, response)
        return response

    def poll(self) -> dict[str, object]:
        if not self.command_path.exists():
            return {"status": "IDLE"}
        command = _read(self.command_path)
        command_id = command.get("command_id")
        generation = command.get("generation")
        if not isinstance(command_id, int) or not isinstance(generation, int):
            return self._response(
                command, "REJECTED", reason="invalid command identity"
            )
        if (
            command.get("run_id") != self.run_id
            or command.get("fixture_id") != self.fixture_id
        ):
            return self._response(command, "REJECTED", reason="foreign command context")
        if command_id == self._last_command_id and self._last_response is not None:
            return dict(self._last_response)
        if command_id <= self._last_command_id or generation <= self._last_command_id:
            return self._response(
                command, "REJECTED", reason="stale or duplicate command"
            )

        operation = command.get("operation")
        handler = self.handlers.get(operation) if isinstance(operation, str) else None
        if handler is None:
            response = self._response(
                command, "REJECTED", reason="unsupported operation"
            )
        else:
            try:
                result = dict(handler(command.get("payload", {})))
            except Exception as error:
                response = self._response(
                    command, "REJECTED", reason=f"command failed closed: {error}"
                )
            else:
                response = self._response(command, "OK", result=result)
        self._last_command_id = command_id
        self._last_response = dict(response)
        return response


@dataclass(frozen=True)
class AmrTopPlaneIdentity:
    prim_path: str
    world_transform: object


class AmrTopPlaneResolver:
    """Resolve one canonical AMR top prim; never search by fuzzy name."""

    def __init__(self, canonical_path: str, matching_paths: list[str] | None = None):
        self.canonical_path = canonical_path
        self.matching_paths = matching_paths

    def resolve(self, stage, world_transform_reader=None) -> AmrTopPlaneIdentity:
        if self.matching_paths is not None and len(self.matching_paths) != 1:
            raise ValueError("multiple AMR top-plane matches")
        prim = stage.GetPrimAtPath(self.canonical_path)
        if prim is None or not prim.IsValid():
            raise ValueError("missing canonical AMR top-plane prim")
        transform = getattr(prim, "world_transform", None)
        if transform is None and world_transform_reader is not None:
            transform = world_transform_reader(prim)
        if transform is None:
            raise ValueError("AMR top-plane world transform is unavailable")
        return AmrTopPlaneIdentity(self.canonical_path, transform)


def local_pose_provenance(
    *,
    support_surface_id: str,
    support_surface_frame: str,
    sampled_pose_amr_top: Mapping[str, float],
    spawn_pose_world: Mapping[str, float],
) -> dict[str, object]:
    if support_surface_frame != "amr_top":
        raise ValueError("unsupported AMR support-surface frame")
    return {
        "support_surface_id": support_surface_id,
        "support_surface_frame": support_surface_frame,
        "sampled_pose_amr_top": {"frame_id": "amr_top", **dict(sampled_pose_amr_top)},
        "spawn_pose_world": {"frame_id": "world", **dict(spawn_pose_world)},
    }


def candidate_identity(
    *, seed: int, index: int, local_pose: Mapping[str, object]
) -> str:
    canonical = json.dumps(
        {"seed": seed, "index": index, "local_pose": dict(local_pose)},
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return f"candidate-{seed}-{index}-{hashlib.sha256(canonical).hexdigest()[:12]}"


def sample_amr_top_candidate(
    *,
    seed: int,
    index: int,
    usable_dimensions: tuple[float, float],
    fixture_dimensions: tuple[float, float, float],
    top_surface_z: float,
    clearance_m: float,
    usable_bounds_local: Mapping[str, float] | None = None,
    sampling_fixture_dimensions: tuple[float, float, float] | None = None,
) -> dict[str, object]:
    """Generate one deterministic AMR-local center, preserving seeded draw order."""
    if index < 0:
        raise ValueError("candidate index must be non-negative")
    width, depth = (float(value) for value in usable_dimensions)
    sampling_dimensions = sampling_fixture_dimensions or fixture_dimensions
    fx, fy, fz = (float(value) for value in sampling_dimensions)
    if min(width, depth, fx, fy, fz) <= 0.0 or clearance_m < 0.0:
        raise ValueError("AMR candidate dimensions and clearance must be valid")
    if usable_bounds_local is not None:
        x_min = float(usable_bounds_local["x_min"])
        x_max = float(usable_bounds_local["x_max"])
        y_min = float(usable_bounds_local["y_min"])
        y_max = float(usable_bounds_local["y_max"])
    else:
        x_min, x_max = -width / 2.0, width / 2.0
        y_min, y_max = -depth / 2.0, depth / 2.0
    rng = random.Random(seed)
    sample = None
    for _ in range(index + 1):
        sample = {
            "x": rng.uniform(x_min + fx / 2.0, x_max - fx / 2.0),
            "y": rng.uniform(y_min + fy / 2.0, y_max - fy / 2.0),
            "z": float(top_surface_z) + fz / 2.0 + float(clearance_m),
            "yaw": rng.uniform(-3.141592653589793, 3.141592653589793),
        }
    assert sample is not None
    return {
        "candidate_id": candidate_identity(seed=seed, index=index, local_pose=sample),
        "seed": seed,
        "index": index,
        "pose": {"frame_id": "amr_top", **sample},
    }
