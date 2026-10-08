"""Pure JSON handoff between Isaac Script Editor and ROS Humble."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import os
import tempfile

from .fixture import ValidationManifest


HANDOFF_SCHEMA_VERSION = "wu14-pnp-registration/v1"
CLEANUP_HANDOFF_SCHEMA_VERSION = "wu14-pnp-cleanup/v1"


def atomic_write_json(path: str | Path, payload: dict[str, object]) -> Path:
    """Atomically replace a validation-only JSON handoff or acknowledgement."""
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return destination


@dataclass(frozen=True)
class RegistrationHandoff:
    """Runtime/debug request consumed by the ROS-side registration helper."""

    run_id: str
    target_id: str
    pose: dict[str, object]
    has_target_yaw: bool
    ttl_s: float
    fixture_prim_identity: str
    seed: int
    profile_reference: str
    dimensions: tuple[float, float, float]
    status: str = "FIXTURE_READY_FOR_REGISTRATION"
    schema_version: str = HANDOFF_SCHEMA_VERSION

    @classmethod
    def from_manifest(cls, manifest: ValidationManifest, profile_reference: str):
        pose = manifest.registered_top_surface_pose_base_link
        return cls(
            run_id=manifest.run_id,
            target_id=manifest.target_id,
            pose={
                "frame_id": pose.frame_id,
                "x": pose.x,
                "y": pose.y,
                "z": pose.z,
                "yaw": pose.yaw,
            },
            has_target_yaw=manifest.has_target_yaw,
            ttl_s=manifest.ttl_s,
            fixture_prim_identity=manifest.fixture_prim_identity,
            seed=manifest.seed,
            profile_reference=profile_reference,
            dimensions=manifest.dimensions,
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "status": self.status,
            "run_id": self.run_id,
            "target_id": self.target_id,
            "request": {
                "run_id": self.run_id,
                "target_id": self.target_id,
                "pose": dict(self.pose),
                "has_target_yaw": self.has_target_yaw,
                "ttl_s": self.ttl_s,
            },
            "fixture": {
                "prim_identity": self.fixture_prim_identity,
                "seed": self.seed,
                "dimensions_m": list(self.dimensions),
            },
            "profile_reference": self.profile_reference,
        }


def atomic_write_registration_handoff(
    path: str | Path, handoff: RegistrationHandoff
) -> Path:
    """Write the runtime seam without exposing a partially written JSON file."""
    return atomic_write_json(path, handoff.to_dict())
