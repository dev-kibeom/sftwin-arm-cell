"""Pure policy and thin runtime seams for the WU-14 validation fixture.

This module intentionally has no Isaac or ROS imports.  Isaac-facing code can
use the values and decisions here without making policy untestable outside the
simulator.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import random
from typing import Iterable, Mapping, Sequence


_EPSILON = 1e-12
SUPPORTED_GEOMETRY_FRAMES = frozenset({"world"})


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass(frozen=True)
class PoseReference(Pose):
    frame_id: str = "base_link"


@dataclass(frozen=True)
class RigidTransform:
    """Yaw-only transform used by the approved Isaac world/base profile."""

    x: float
    y: float
    z: float
    yaw: float

    @classmethod
    def from_world_to_base(cls, values: Sequence[float]) -> "RigidTransform":
        if len(values) != 4:
            raise ValueError("world_to_base must contain x, y, z, and yaw")
        return cls(*(float(value) for value in values))

    def base_to_world(self, pose: Pose) -> Pose:
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return Pose(
            self.x + cosine * pose.x - sine * pose.y,
            self.y + sine * pose.x + cosine * pose.y,
            self.z + pose.z,
            _normalize_angle(self.yaw + pose.yaw),
        )

    def world_to_base(self, pose: Pose) -> Pose:
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        dx = pose.x - self.x
        dy = pose.y - self.y
        return Pose(
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            pose.z - self.z,
            _normalize_angle(pose.yaw - self.yaw),
        )


def _normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Aabb:
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    def __post_init__(self):
        if len(self.minimum) != 3 or len(self.maximum) != 3:
            raise ValueError("AABB bounds must have exactly three coordinates")
        if any(not math.isfinite(value) for value in (*self.minimum, *self.maximum)):
            raise ValueError("AABB bounds must be finite")
        if any(low > high for low, high in zip(self.minimum, self.maximum)):
            raise ValueError("AABB minimum must not exceed maximum")

    def contains_center(self, center: Pose, dimensions: Sequence[float]) -> bool:
        half = _half_extents(dimensions)
        return all(
            low + extent <= coordinate <= high - extent
            for coordinate, low, high, extent in zip(
                (center.x, center.y, center.z), self.minimum, self.maximum, half
            )
        )

    def intersects_center(self, center: Pose, dimensions: Sequence[float]) -> bool:
        half = _half_extents(dimensions)
        center_bounds = tuple(
            (coordinate - extent, coordinate + extent)
            for coordinate, extent in zip((center.x, center.y, center.z), half)
        )
        return all(
            object_min < volume_max and object_max > volume_min
            for (object_min, object_max), volume_min, volume_max in zip(
                center_bounds, self.minimum, self.maximum
            )
        )


@dataclass(frozen=True)
class FixtureProfile:
    target_id: str
    fixture_dimensions: tuple[float, float, float]
    spawn_bounds: Aabb
    excluded_volumes: tuple[Aabb, ...]
    seed: int
    expected_fixture_release_position_base: tuple[float, float, float]
    place_position_tolerance_m: float
    geometry_frame: str = "world"
    orientation_tolerance_rad: float = math.radians(10.0)
    registration_ttl_s: float = 30.0
    max_registration_ttl_s: float = 60.0
    deletion_delay_s: float = 3.0
    holding_freshness_ms: float = 500.0
    open_readiness_timeout_s: float = 2.0
    has_target_yaw: bool = True
    capture_profile_reference: str = "pnp_validation_profile.yaml:capture"
    tolerance_profile_reference: str = "pnp_validation_profile.yaml:tolerances"
    support_surface_id: str | None = None
    support_surface_frame: str | None = None
    spawn_clearance_m: float = 0.001

    def __post_init__(self):
        _positive_dimensions(self.fixture_dimensions, "fixture dimensions")
        if not self.target_id:
            raise ValueError("target_id must not be empty")
        if not 0.0 < self.registration_ttl_s <= self.max_registration_ttl_s:
            raise ValueError("registration TTL must be in the configured range")
        if self.max_registration_ttl_s > 60.0:
            raise ValueError("registration TTL maximum must not exceed 60 seconds")
        if self.deletion_delay_s < 0.0:
            raise ValueError("deletion delay must not be negative")
        if self.holding_freshness_ms <= 0.0:
            raise ValueError("holding freshness must be positive")
        if not math.isfinite(self.open_readiness_timeout_s) or (
            self.open_readiness_timeout_s <= 0.0
        ):
            raise ValueError("open readiness timeout must be positive and finite")
        if not math.isfinite(self.orientation_tolerance_rad) or (
            self.orientation_tolerance_rad < 0.0
        ):
            raise ValueError("orientation tolerance must be finite and non-negative")
        if self.geometry_frame not in SUPPORTED_GEOMETRY_FRAMES:
            raise ValueError(f"unsupported geometry_frame: {self.geometry_frame}")
        if len(self.expected_fixture_release_position_base) != 3 or any(
            not math.isfinite(value)
            for value in self.expected_fixture_release_position_base
        ):
            raise ValueError("expected fixture release position must be finite xyz")
        if (
            not math.isfinite(self.place_position_tolerance_m)
            or self.place_position_tolerance_m <= 0.0
        ):
            raise ValueError("PLACE position tolerance must be positive and finite")
        if self.support_surface_id is not None:
            if not self.support_surface_id.startswith("/"):
                raise ValueError("support_surface_id must be a canonical prim path")
            if self.support_surface_frame != "amr_top":
                raise ValueError("support_surface_frame must be amr_top")
        if not math.isfinite(self.spawn_clearance_m) or self.spawn_clearance_m < 0.0:
            raise ValueError("spawn_clearance_m must be finite and non-negative")


def _positive_dimensions(dimensions: Sequence[float], label: str) -> None:
    if len(dimensions) != 3 or any(
        not math.isfinite(value) or value <= 0.0 for value in dimensions
    ):
        raise ValueError(f"{label} must contain three positive finite values")


def _half_extents(dimensions: Sequence[float]) -> tuple[float, float, float]:
    _positive_dimensions(dimensions, "dimensions")
    return tuple(value / 2.0 for value in dimensions)


def sample_spawn_pose(profile: FixtureProfile, *, max_attempts: int = 1000) -> Pose:
    """Return the first valid seeded sample, using a stable draw order."""
    if max_attempts <= 0:
        raise ValueError("max_attempts must be positive")
    bounds = profile.spawn_bounds
    half = _half_extents(profile.fixture_dimensions)
    low = tuple(minimum + extent for minimum, extent in zip(bounds.minimum, half))
    high = tuple(maximum - extent for maximum, extent in zip(bounds.maximum, half))
    if any(lower > upper for lower, upper in zip(low, high)):
        raise ValueError("fixture dimensions do not fit inside spawn bounds")

    rng = random.Random(profile.seed)
    for _ in range(max_attempts):
        pose = Pose(
            rng.uniform(low[0], high[0]),
            rng.uniform(low[1], high[1]),
            rng.uniform(low[2], high[2]),
            rng.uniform(-math.pi, math.pi),
        )
        if not any(
            excluded.intersects_center(pose, profile.fixture_dimensions)
            for excluded in profile.excluded_volumes
        ):
            return pose
    raise ValueError("unable to sample a valid fixture pose within max_attempts")


def top_surface_reference(
    spawn_pose: Pose, dimensions: Sequence[float], *, frame_id: str = "base_link"
) -> PoseReference:
    """Derive the visible top-surface centroid from actual cube pose/dimensions."""
    if not frame_id:
        raise ValueError("frame_id must not be empty")
    height = _positive_height(dimensions)
    return PoseReference(
        spawn_pose.x,
        spawn_pose.y,
        spawn_pose.z + height / 2.0,
        spawn_pose.yaw,
        frame_id,
    )


def _positive_height(dimensions: Sequence[float]) -> float:
    _positive_dimensions(dimensions, "dimensions")
    return float(dimensions[2])


@dataclass
class FixtureOwnership:
    owned_prim_paths: set[str] = field(default_factory=set)

    def claim(self, prim_path: str) -> None:
        if not prim_path:
            raise ValueError("fixture prim identity must not be empty")
        self.owned_prim_paths.add(prim_path)

    def owns(self, prim_path: str) -> bool:
        return prim_path in self.owned_prim_paths

    def release(self, prim_path: str) -> None:
        self.owned_prim_paths.discard(prim_path)

    def abort_cleanup(self, existing_prim_paths: Iterable[str]) -> list[str]:
        return [path for path in existing_prim_paths if self.owns(path)]


@dataclass(frozen=True)
class CleanupDecision:
    eligible: bool
    deadline_s: float | None = None
    reason: str = ""
    expected_position: tuple[float, float, float] | None = None
    actual_position: tuple[float, float, float] | None = None
    position_error_m: float | None = None
    position_tolerance_m: float | None = None
    post_release_position_observation: str = "UNAVAILABLE"

    def is_due(self, now_s: float) -> bool:
        return (
            self.eligible and self.deadline_s is not None and now_s >= self.deadline_s
        )


def cleanup_decision(
    ownership: FixtureOwnership,
    *,
    prim_path: str,
    place_succeeded: bool,
    fresh_released: bool,
    detach_confirmed: bool,
    ownership_unambiguous: bool,
    actual_pose: Pose | None,
    expected_position: Sequence[float],
    position_tolerance_m: float,
    now_s: float = 0.0,
    delay_s: float = 3.0,
) -> CleanupDecision:
    if not ownership.owns(prim_path):
        return CleanupDecision(False, reason="prim is not fixture-owned")
    if not place_succeeded:
        return CleanupDecision(False, reason="PLACE did not succeed")
    if not fresh_released:
        return CleanupDecision(False, reason="fresh RELEASED was not confirmed")
    if not detach_confirmed:
        return CleanupDecision(False, reason="fixture is not confirmed detached")
    if not ownership_unambiguous:
        return CleanupDecision(False, reason="fixture ownership is ambiguous")
    expected = None
    actual = None
    error = None
    observation = "UNAVAILABLE"
    valid_pose = actual_pose is not None and all(
        math.isfinite(value) for value in (actual_pose.x, actual_pose.y, actual_pose.z)
    )
    valid_expected = len(expected_position) == 3 and all(
        math.isfinite(value) for value in expected_position
    )
    valid_tolerance = math.isfinite(position_tolerance_m) and position_tolerance_m > 0.0
    if valid_pose and valid_expected and valid_tolerance:
        expected = tuple(float(value) for value in expected_position)
        actual = (actual_pose.x, actual_pose.y, actual_pose.z)
        error = math.sqrt(sum((a - e) ** 2 for a, e in zip(actual, expected)))
        observation = "PASS" if error <= position_tolerance_m else "OUTSIDE_TOLERANCE"
    if not math.isfinite(now_s) or not math.isfinite(delay_s) or delay_s < 0.0:
        raise ValueError("cleanup scheduling times must be finite and non-negative")
    return CleanupDecision(
        True,
        deadline_s=now_s + delay_s,
        expected_position=expected,
        actual_position=actual,
        position_error_m=error,
        position_tolerance_m=position_tolerance_m if valid_tolerance else None,
        post_release_position_observation=observation,
    )


def fixture_center_from_tcp_target(
    *,
    tcp_position: Sequence[float],
    tcp_quaternion_xyzw: Sequence[float],
    object_to_tcp_translation: Sequence[float],
    object_to_tcp_quaternion_xyzw: Sequence[float],
) -> tuple[float, float, float]:
    """Invert the motion layer's T_base_tcp = T_base_object * T_object_tcp."""
    dimensions = (
        (tcp_position, 3),
        (tcp_quaternion_xyzw, 4),
        (object_to_tcp_translation, 3),
        (object_to_tcp_quaternion_xyzw, 4),
    )
    if any(len(values) != expected for values, expected in dimensions):
        raise ValueError("TCP/object transform has an invalid dimension")
    values = (
        *tcp_position,
        *tcp_quaternion_xyzw,
        *object_to_tcp_translation,
        *object_to_tcp_quaternion_xyzw,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("TCP/object transform must be finite")
    q_tcp = _normalize_quaternion(tcp_quaternion_xyzw)
    q_ot = _normalize_quaternion(object_to_tcp_quaternion_xyzw)
    q_to = (-q_ot[0], -q_ot[1], -q_ot[2], q_ot[3])
    q_object = _quaternion_multiply(q_tcp, q_to)
    rotated_offset = _rotate_vector(q_object, object_to_tcp_translation)
    return tuple(float(tcp_position[i]) - rotated_offset[i] for i in range(3))


def _normalize_quaternion(
    values: Sequence[float],
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm <= _EPSILON:
        raise ValueError("quaternion must be non-zero")
    return tuple(float(value) / norm for value in values)


def _quaternion_multiply(
    a: Sequence[float], b: Sequence[float]
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(
    q: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    pure = (float(vector[0]), float(vector[1]), float(vector[2]), 0.0)
    inverse = (-q[0], -q[1], -q[2], q[3])
    return _quaternion_multiply(_quaternion_multiply(q, pure), inverse)[:3]


@dataclass(frozen=True, init=False)
class ValidationManifest:
    run_id: str
    seed: int
    target_id: str
    fixture_prim_identity: str
    dimensions: tuple[float, float, float]
    spawn_pose_world: PoseReference
    registered_top_surface_pose_base_link: PoseReference
    has_target_yaw: bool
    ttl_s: float
    capture_profile_reference: str
    tolerance_profile_reference: str
    support_surface_id: str | None = None
    support_surface_frame: str | None = None
    sampled_pose_amr_top: Mapping[str, object] | None = None
    support_surface_world_transform: object | None = None

    def __init__(
        self,
        *,
        run_id: str,
        seed: int,
        target_id: str,
        fixture_prim_identity: str,
        dimensions: tuple[float, float, float],
        spawn_pose_world: PoseReference | Pose | None = None,
        registered_top_surface_pose_base_link: PoseReference | None = None,
        has_target_yaw: bool,
        ttl_s: float,
        capture_profile_reference: str,
        tolerance_profile_reference: str,
        support_surface_id: str | None = None,
        support_surface_frame: str | None = None,
        sampled_pose_amr_top: Mapping[str, object] | None = None,
        support_surface_world_transform: object | None = None,
        spawn_pose: PoseReference | Pose | None = None,
        registered_top_surface_pose: PoseReference | None = None,
    ):
        if spawn_pose_world is None:
            spawn_pose_world = spawn_pose
        if registered_top_surface_pose_base_link is None:
            registered_top_surface_pose_base_link = registered_top_surface_pose
        if spawn_pose_world is None or registered_top_surface_pose_base_link is None:
            raise ValueError(
                "manifest must include explicit spawn and registration poses"
            )
        if not isinstance(spawn_pose_world, PoseReference):
            spawn_pose_world = PoseReference(
                spawn_pose_world.x,
                spawn_pose_world.y,
                spawn_pose_world.z,
                spawn_pose_world.yaw,
                "world",
            )
        if not isinstance(registered_top_surface_pose_base_link, PoseReference):
            registered_top_surface_pose_base_link = PoseReference(
                registered_top_surface_pose_base_link.x,
                registered_top_surface_pose_base_link.y,
                registered_top_surface_pose_base_link.z,
                registered_top_surface_pose_base_link.yaw,
                "base_link",
            )
        object.__setattr__(self, "run_id", run_id)
        object.__setattr__(self, "seed", seed)
        object.__setattr__(self, "target_id", target_id)
        object.__setattr__(self, "fixture_prim_identity", fixture_prim_identity)
        object.__setattr__(self, "dimensions", dimensions)
        object.__setattr__(self, "spawn_pose_world", spawn_pose_world)
        object.__setattr__(
            self,
            "registered_top_surface_pose_base_link",
            registered_top_surface_pose_base_link,
        )
        object.__setattr__(self, "has_target_yaw", has_target_yaw)
        object.__setattr__(self, "ttl_s", ttl_s)
        object.__setattr__(self, "capture_profile_reference", capture_profile_reference)
        object.__setattr__(
            self, "tolerance_profile_reference", tolerance_profile_reference
        )
        object.__setattr__(self, "support_surface_id", support_surface_id)
        object.__setattr__(self, "support_surface_frame", support_surface_frame)
        object.__setattr__(self, "sampled_pose_amr_top", sampled_pose_amr_top)
        object.__setattr__(
            self, "support_surface_world_transform", support_surface_world_transform
        )

    @classmethod
    def from_fixture(
        cls,
        *,
        run_id: str,
        profile: FixtureProfile,
        prim_path: str,
        spawn_pose_world: Pose,
        registered_top_surface_pose_base_link: PoseReference,
    ) -> "ValidationManifest":
        if not run_id:
            raise ValueError("run_id must not be empty")
        return cls(
            run_id=run_id,
            seed=profile.seed,
            target_id=profile.target_id,
            fixture_prim_identity=prim_path,
            dimensions=profile.fixture_dimensions,
            spawn_pose_world=PoseReference(
                spawn_pose_world.x,
                spawn_pose_world.y,
                spawn_pose_world.z,
                spawn_pose_world.yaw,
                profile.geometry_frame,
            ),
            registered_top_surface_pose_base_link=registered_top_surface_pose_base_link,
            has_target_yaw=profile.has_target_yaw,
            ttl_s=profile.registration_ttl_s,
            capture_profile_reference=profile.capture_profile_reference,
            tolerance_profile_reference=profile.tolerance_profile_reference,
            support_surface_id=profile.support_surface_id,
            support_surface_frame=profile.support_surface_frame,
        )

    def to_dict(self) -> Mapping[str, object]:
        data = {
            "run_id": self.run_id,
            "seed": self.seed,
            "target_id": self.target_id,
            "fixture_prim_identity": self.fixture_prim_identity,
            "dimensions": list(self.dimensions),
            "spawn_pose_world": {
                **_pose_dict(self.spawn_pose_world),
                "frame_id": self.spawn_pose_world.frame_id,
            },
            "registered_top_surface_pose_base_link": {
                **_pose_dict(self.registered_top_surface_pose_base_link),
                "frame_id": self.registered_top_surface_pose_base_link.frame_id,
            },
            "has_target_yaw": self.has_target_yaw,
            "ttl_s": self.ttl_s,
            "capture_profile_reference": self.capture_profile_reference,
            "tolerance_profile_reference": self.tolerance_profile_reference,
        }
        if self.support_surface_id is not None:
            data["support_surface_id"] = self.support_surface_id
            data["support_surface_frame"] = self.support_surface_frame
        if self.sampled_pose_amr_top is not None:
            data["sampled_pose_amr_top"] = dict(self.sampled_pose_amr_top)
        if self.support_surface_world_transform is not None:
            data["support_surface_world_transform"] = (
                self.support_surface_world_transform
            )
        return data

    @property
    def spawn_pose(self) -> PoseReference:
        """Compatibility alias; serialized manifests use the explicit name."""
        return self.spawn_pose_world

    @property
    def registered_top_surface_pose(self) -> PoseReference:
        """Compatibility alias; serialized manifests use the explicit name."""
        return self.registered_top_surface_pose_base_link


def _pose_dict(pose: Pose) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "z": pose.z, "yaw": pose.yaw}
