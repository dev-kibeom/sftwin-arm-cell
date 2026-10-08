"""Validation-only orchestration helper for one deterministic fixture run.

The helper owns fixture material and registration context.  It delegates the
mission itself to the existing Orchestration runtime and never implements a
PICK/PLACE/GO_HOME state machine.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
from typing import Callable, Protocol

import yaml

from .fixture import (
    Aabb,
    CleanupDecision,
    FixtureOwnership,
    FixtureProfile,
    Pose,
    RigidTransform,
    ValidationManifest,
    cleanup_decision,
    fixture_center_from_tcp_target,
    sample_spawn_pose,
    SUPPORTED_GEOMETRY_FRAMES,
    top_surface_reference,
)
from .handoff import CLEANUP_HANDOFF_SCHEMA_VERSION, atomic_write_json
from .live_fixture import (
    AmrTopPlaneResolver,
    FixtureRuntimeConsumer,
    cube_geometry_from_transform,
    sample_amr_top_candidate,
)
from gripper_runtime.grasp_attachment import CaptureReferenceConfig
from gripper_runtime.grasp_policy import CaptureConfig
from gripper_runtime.robotiq_actuator import Robotiq2F85Actuator


CANONICAL_AMR_TOP_PLANE_PATH = "/World/SF_Twin_Cell/AMR/Mockup/Tray"
MISSION_COMPLETION_SCHEMA_VERSION = "wu14-pnp-mission-completion/v1"


class ValidationStartPending(RuntimeError):
    """A physical OPEN check is waiting for a transient condition."""


class ValidationStartRejected(RuntimeError):
    """A validation start precondition failed terminally."""


def validation_start_diagnostic(grasp_manager, baseline_generation, reason):
    """Return bounded diagnostics for a physical OPEN readiness decision."""
    gripper = getattr(grasp_manager, "gripper", None)
    observation = getattr(grasp_manager, "holding_observation", lambda: None)()
    get_position = getattr(gripper, "get_position", None)
    position = None
    if callable(get_position):
        try:
            position = float(get_position())
        except (TypeError, ValueError, RuntimeError):
            position = None
    derived_width = None
    if position is not None and math.isfinite(position):
        derived_width = Robotiq2F85Actuator.position_to_width(position)
    observed_at = getattr(observation, "observed_at", None)
    freshness = None
    if observation is not None:
        try:
            freshness = bool(observation.fresh())
        except (AttributeError, TypeError, ValueError):
            freshness = None
    return {
        "reason": str(reason),
        "baseline_feedback_generation": baseline_generation,
        "current_feedback_generation": getattr(gripper, "feedback_generation", None),
        "is_moving": bool(getattr(gripper, "is_moving", False)),
        "parent_q": position,
        "derived_opening_width_mm": derived_width,
        "holding_state": getattr(getattr(observation, "state", None), "value", None),
        "holding_fresh": freshness,
        "holding_observed_at": observed_at,
        "attached": bool(getattr(grasp_manager, "attached", False))
        or bool(getattr(observation, "attached", False)),
        "owned_joint": getattr(grasp_manager, "attach_joint_path", None),
    }


def capture_config_from_profile(profile: dict) -> CaptureConfig:
    """Load Task 3's canonical CaptureConfig without unit conversion."""
    capture = profile.get("capture", {})
    if "orientation_tolerance_rad" in capture:
        raise ValueError(
            "validation capture profile must use orientation_tolerance_deg"
        )
    return CaptureConfig(
        capture_volume_dimensions_m=tuple(capture["volume_dimensions_m"]),
        position_tolerance_m=float(capture["position_tolerance_m"]),
        orientation_tolerance_deg=float(capture["orientation_tolerance_deg"]),
        expected_contact_width_mm=float(capture["expected_contact_width_mm"]),
        contact_width_tolerance_mm=float(capture["contact_width_tolerance_mm"]),
    ).validate()


def capture_reference_config_from_profile(profile: dict) -> CaptureReferenceConfig:
    """Load the validation fixture geometry used by Isaac capture queries."""
    fixture = profile["fixture"]
    motion = profile["motion"]
    observation_to_object = motion["observation_to_object_translation"]
    translation = motion["object_to_grasp_tcp_translation"]
    if len(observation_to_object) != 3 or len(translation) != 3:
        raise ValueError(
            "observation and object grasp transforms must have three values"
        )
    config = CaptureReferenceConfig(
        fixture_dimensions_m=tuple(float(value) for value in fixture["dimensions_m"]),
        observation_to_grasp_tcp_m=float(observation_to_object[2])
        + float(translation[2]),
        insertion_axis_tcp=tuple(
            float(value) for value in motion["insertion_axis_tcp"]
        ),
    )
    return config.validate()


def prepare_validation_start_state(grasp_manager) -> bool:
    """Open and validate the gripper before validation material is created.

    This synchronous helper is primarily used by tests and diagnostics.  The
    live Script Editor entrypoint uses ``request_validation_start_state`` and
    ``validation_start_state_ready`` across physics ticks so the joint sample
    is demonstrably newer than the OPEN command.
    """
    baseline_generation = request_validation_start_state(grasp_manager)
    return validation_start_state_ready(grasp_manager, baseline_generation)


def request_validation_start_state(grasp_manager):
    """Issue OPEN after rejecting stale attachment state.

    Returns the last observed physics feedback generation.  The caller must
    wait for a strictly newer generation before accepting physical OPEN.
    """
    if getattr(grasp_manager, "attach_joint_path", None):
        raise ValidationStartRejected(
            "validation start rejected: existing owned grasp joint"
        )
    gripper = getattr(grasp_manager, "gripper", None)
    opener = getattr(gripper, "open", None)
    if not callable(opener):
        raise ValidationStartRejected(
            "validation start rejected: gripper open unavailable"
        )
    baseline_generation = getattr(gripper, "feedback_generation", None)
    try:
        result = opener()
    except Exception as error:
        raise ValidationStartRejected("validation start open command failed") from error
    if result is False:
        raise ValidationStartRejected("validation start open command failed")
    return baseline_generation


def validation_start_state_ready(grasp_manager, baseline_generation) -> bool:
    """Return true only after fresh physical feedback confirms OPEN.

    ``released`` is intentionally checked separately: it describes attachment
    ownership, not finger opening.  The authoritative physical state is the
    Isaac/ROS parent joint represented by the gripper's ``get_position``.
    """
    gripper = getattr(grasp_manager, "gripper", None)
    observation = getattr(grasp_manager, "holding_observation", lambda: None)()
    if getattr(observation, "attached", False) or getattr(
        grasp_manager, "attached", False
    ):
        raise ValidationStartRejected("validation start rejected: gripper is attached")
    if (
        observation is None
        or getattr(getattr(observation, "state", None), "value", None) == "unknown"
    ):
        raise ValidationStartRejected(
            "validation start requires fresh RELEASED observation"
        )
    fresh = getattr(observation, "fresh", None)
    if not callable(fresh):
        raise ValidationStartRejected(
            "validation start requires fresh RELEASED observation"
        )
    try:
        observation_is_fresh = bool(fresh())
    except (AttributeError, TypeError, ValueError, RuntimeError) as error:
        raise ValidationStartRejected(
            "validation start requires fresh RELEASED observation"
        ) from error
    if (
        not observation_is_fresh
        or getattr(getattr(observation, "state", None), "value", None) != "released"
    ):
        raise ValidationStartRejected(
            "validation start requires fresh RELEASED observation"
        )
    if getattr(grasp_manager, "attach_joint_path", None):
        raise ValidationStartRejected(
            "validation start rejected: existing owned grasp joint"
        )

    get_position = getattr(gripper, "get_position", None)
    moving_marker = object()
    moving = getattr(gripper, "is_moving", moving_marker)
    if gripper is None or not callable(get_position) or moving is moving_marker:
        raise ValidationStartRejected(
            "validation start requires physical gripper feedback"
        )
    if moving is None:
        raise ValidationStartRejected(
            "validation start requires physical motion state feedback"
        )
    generation = getattr(gripper, "feedback_generation", None)
    if baseline_generation is None or generation is None:
        raise ValidationStartRejected(
            "validation start requires physical feedback generation"
        )
    try:
        baseline_generation = int(baseline_generation)
        generation = int(generation)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValidationStartRejected(
            "validation start requires valid physical feedback generation"
        ) from error

    if generation <= baseline_generation:
        raise ValidationStartPending(
            "validation start requires fresh physical joint-state feedback"
        )

    if bool(moving):
        raise ValidationStartPending(
            "validation start requires physical OPEN to settle"
        )

    try:
        position = float(get_position())
    except (TypeError, ValueError, RuntimeError, OverflowError) as error:
        raise ValidationStartRejected(
            "validation start requires physical gripper feedback"
        ) from error

    if not math.isfinite(position):
        raise ValidationStartRejected(
            "validation start requires finite physical OPEN feedback"
        )
    open_position_tolerance = Robotiq2F85Actuator.POSITION_EPSILON
    if position > open_position_tolerance:
        raise ValidationStartRejected(
            "validation start requires physical OPEN parent joint"
        )
    return True


def validate_validation_profile(profile: dict) -> dict:
    """Reject validation recipes that cannot fit the configured 2F-85 aperture."""
    capture_config_from_profile(profile)
    open_readiness_timeout_s = float(
        profile.get("validation", {}).get("open_readiness_timeout_s", 2.0)
    )
    if not math.isfinite(open_readiness_timeout_s) or open_readiness_timeout_s <= 0.0:
        raise ValueError("validation open_readiness_timeout_s must be positive")
    fixture = profile["fixture"]
    geometry_frame = fixture.get("geometry_frame")
    support_surface_id = fixture.get("support_surface_id", CANONICAL_AMR_TOP_PLANE_PATH)
    support_surface_frame = fixture.get("support_surface_frame", "amr_top")
    if geometry_frame not in SUPPORTED_GEOMETRY_FRAMES:
        raise ValueError("fixture geometry_frame must be explicitly supported")
    place = profile.get("place", {})
    tolerance = float(place.get("position_tolerance_m", 0.0))
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("PLACE position_tolerance_m must be positive and finite")
    if not isinstance(support_surface_id, str) or not support_surface_id.startswith(
        "/"
    ):
        raise ValueError("fixture support_surface_id must be a canonical prim path")
    if support_surface_frame != "amr_top":
        raise ValueError("fixture support_surface_frame must be amr_top")
    dimensions_m = tuple(float(value) for value in fixture["dimensions_m"])
    if len(dimensions_m) != 3 or any(value <= 0.0 for value in dimensions_m):
        raise ValueError("fixture dimensions must contain three positive values")

    # The deterministic top-grasp recipe uses the fixture X dimension as its
    # declared grasp dimension; X and Y are equal in the validation profile.
    grasp_dimension_mm = dimensions_m[0] * 1000.0
    if grasp_dimension_mm > Robotiq2F85Actuator.MAX_WIDTH_MM:
        raise ValueError("fixture grasp dimension exceeds Robotiq 2F-85 opening")
    expected_width_mm = float(profile["capture"]["expected_contact_width_mm"])
    if not math.isclose(grasp_dimension_mm, expected_width_mm, abs_tol=1e-9):
        raise ValueError(
            "fixture grasp dimension and capture expected contact width must match"
        )
    motion = profile["motion"]
    observation_translation = motion.get("observation_to_object_translation")
    if (
        not isinstance(observation_translation, list)
        or len(observation_translation) != 3
        or not all(math.isfinite(float(value)) for value in observation_translation)
    ):
        raise ValueError(
            "validation observation_to_object_translation must have three finite values"
        )
    observation_reference = motion.get("observation_reference")
    if not isinstance(observation_reference, str) or not observation_reference:
        raise ValueError("validation fixture observation reference is not declared")
    if not math.isclose(float(observation_translation[0]), 0.0, abs_tol=1e-9):
        raise ValueError("validation observation_to_object_translation x must be zero")
    if not math.isclose(float(observation_translation[1]), 0.0, abs_tol=1e-9):
        raise ValueError("validation observation_to_object_translation y must be zero")
    if not math.isclose(
        float(observation_translation[2]),
        -float(fixture["dimensions_m"][2]) * 0.5,
        abs_tol=1e-9,
    ):
        raise ValueError(
            "validation observation reference must resolve to geometric center"
        )
    grasp_translation = motion.get("object_to_grasp_tcp_translation")
    if (
        not isinstance(grasp_translation, list)
        or len(grasp_translation) != 3
        or not all(math.isfinite(float(value)) for value in grasp_translation)
    ):
        raise ValueError(
            "validation object_to_grasp_tcp_translation must have three finite values"
        )
    if not math.isclose(float(grasp_translation[0]), 0.0, abs_tol=1e-9):
        raise ValueError("validation object_to_grasp_tcp_translation x must be zero")
    if not math.isclose(float(grasp_translation[1]), 0.0, abs_tol=1e-9):
        raise ValueError("validation object_to_grasp_tcp_translation y must be zero")
    if motion.get("object_to_grasp_tcp_quaternion") != [1.0, 0.0, 0.0, 0.0]:
        raise ValueError(
            "validation tool convention must use the canonical TCP X-180 quaternion"
        )
    if motion.get("insertion_axis_tcp") != [0.0, 0.0, 1.0]:
        raise ValueError(
            "validation insertion_axis_tcp must be the physical TCP +Z axis"
        )
    if not math.isclose(
        float(motion.get("pick_approach_distance_m")), 0.05, abs_tol=1e-9
    ):
        raise ValueError("validation pick_approach_distance_m must remain 0.05 m")
    recipe = fixture.get("grasp_recipe")
    if not isinstance(recipe, dict):
        raise ValueError("validation fixture grasp_recipe is required")
    band = recipe.get("measured_contact_band_m")
    selected_offset = recipe.get("object_top_to_grasp_tcp_z_m")
    if (
        not isinstance(band, list)
        or len(band) != 2
        or not all(math.isfinite(float(value)) for value in band)
        or float(band[0]) >= float(band[1])
        or not math.isfinite(float(selected_offset))
        or not float(band[0]) <= float(selected_offset) <= float(band[1])
    ):
        raise ValueError("validation grasp recipe contact band is invalid")
    if not math.isclose(
        float(observation_translation[2]) + float(grasp_translation[2]),
        float(selected_offset),
        abs_tol=1e-9,
    ):
        raise ValueError(
            "validation motion grasp offset must match fixture recipe offset"
        )
    if motion.get("validation_grasp_contact_policy_enabled") is not True:
        raise ValueError("validation grasp contact policy must be explicitly enabled")
    policy_links = motion.get("validation_grasp_contact_links")
    if policy_links != [
        "gripper_robotiq_85_left_finger_tip_link",
        "gripper_robotiq_85_right_finger_tip_link",
    ]:
        raise ValueError("validation grasp policy must allow only both fingertip links")
    prefix = motion.get("validation_grasp_contact_object_id_prefix")
    if not isinstance(prefix, str) or not prefix.startswith("wu14_fixture_"):
        raise ValueError("validation grasp policy fixture prefix is invalid")
    return profile


def _angular_distance(first: float, second: float) -> float:
    return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)


def load_validation_profile(path: str | Path) -> dict:
    profile_path = Path(path)
    profile = yaml.safe_load(profile_path.read_text()) or {}
    orchestration = profile.get("orchestration", {})
    target_id = orchestration.get("mission_target_id")
    recipe_directory = orchestration.get("recipe_directory", "recipes")
    if not isinstance(target_id, str) or not target_id:
        raise ValueError("validation profile must select a mission target_id")
    recipe_path = profile_path.parent / recipe_directory / f"{target_id}.json"
    profile["_mission_recipe"] = json.loads(recipe_path.read_text())
    return validate_validation_profile(profile)


def fixture_profile_from_validation_config(config: dict) -> FixtureProfile:
    """Build the pure fixture profile shared by Isaac validation entrypoints."""
    fixture = config["fixture"]
    bounds = fixture["spawn_bounds_m"]
    orchestration = config["orchestration"]
    recipe = config["_mission_recipe"]
    motion = config["motion"]
    expected_base = fixture_center_from_tcp_target(
        tcp_position=recipe["place"]["desired_object_pose"]["position"],
        tcp_quaternion_xyzw=recipe["place"]["desired_object_pose"]["orientation_xyzw"],
        object_to_tcp_translation=motion["object_to_grasp_tcp_translation"],
        object_to_tcp_quaternion_xyzw=motion["object_to_grasp_tcp_quaternion"],
    )
    return FixtureProfile(
        target_id=recipe["target_id"],
        fixture_dimensions=tuple(fixture["dimensions_m"]),
        spawn_bounds=Aabb(tuple(bounds["min"]), tuple(bounds["max"])),
        excluded_volumes=tuple(
            Aabb(tuple(volume["min"]), tuple(volume["max"]))
            for volume in fixture["excluded_volumes_m"]
        ),
        seed=int(fixture["seed"]),
        expected_fixture_release_position_base=expected_base,
        place_position_tolerance_m=float(config["place"]["position_tolerance_m"]),
        geometry_frame=fixture["geometry_frame"],
        orientation_tolerance_rad=math.radians(
            float(config["capture"]["orientation_tolerance_deg"])
        ),
        registration_ttl_s=float(config["validation"]["registration_ttl_s"]),
        max_registration_ttl_s=float(config["validation"]["registration_ttl_max_s"]),
        deletion_delay_s=float(config["place"]["deletion_delay_s"]),
        holding_freshness_ms=float(
            config["validation"].get("holding_freshness_ms", 500.0)
        ),
        open_readiness_timeout_s=float(
            config["validation"].get("open_readiness_timeout_s", 2.0)
        ),
        support_surface_id=fixture.get(
            "support_surface_id", CANONICAL_AMR_TOP_PLANE_PATH
        ),
        support_surface_frame=fixture.get("support_surface_frame", "amr_top"),
        spawn_clearance_m=float(fixture.get("spawn_clearance_m", 0.001)),
    )


class FixtureRuntime(Protocol):
    """Thin Isaac-facing seam injected by the live runner."""

    def spawn_cube(
        self, prim_path: str, pose: Pose, dimensions: tuple[float, float, float]
    ) -> None: ...

    def disable_gravity(self, prim_path: str) -> None: ...

    def enable_gravity(self, prim_path: str) -> None: ...

    def register_eligible(self, prim_path: str, run_id: str) -> None: ...

    def retire_fixture_for_pending_delete(self, prim_path: str) -> bool: ...

    def read_base_pose(self, prim_path: str) -> Pose: ...

    def read_world_pose(self, prim_path: str) -> Pose: ...

    def existing_prim_paths(self) -> list[str]: ...

    def delete_prim(self, prim_path: str) -> None: ...


class PreCaptureStabilityGuard:
    """Validation-only ownership guard for the pre-capture fixture phase."""

    def __init__(self, stage, prim_path, rigid_body_api_factory):
        self.stage = stage
        self.prim_path = str(prim_path)
        self._rigid_body_api_factory = rigid_body_api_factory
        self.is_active = False

    def _attribute(self):
        prim = self.stage.GetPrimAtPath(self.prim_path)
        if not prim.IsValid():
            return None
        try:
            rigid_body_api = self._rigid_body_api_factory(prim)
            if rigid_body_api is None:
                return None
            return rigid_body_api.CreateKinematicEnabledAttr()
        except Exception:
            return None

    def hold_for_registration(self):
        attribute = self._attribute()
        if attribute is None:
            return False
        try:
            attribute.Set(True)
        except Exception:
            return False
        self.is_active = True
        return True

    def release_for_capture(self, target_path):
        if str(target_path) != self.prim_path or not self.is_active:
            return False
        attribute = self._attribute()
        if attribute is None:
            return False
        try:
            attribute.Set(False)
        except Exception:
            return False
        self.is_active = False
        return True


class IsaacFixtureRuntime:
    """Isaac-backed owner for exactly one validation cube.

    Isaac imports are lazy so all policy and adapter contracts remain unit-testable
    on a normal Python interpreter.  The eligible registry and Fixed Vision
    registrar are injected because both are owned by existing Task 3 runtimes.
    """

    OWNER_KEY = "sf_twin:validation_fixture_owner"
    OWNER_VALUE = "wu14_validation_fixture"
    AMR_TOP_PLANE_PATH = CANONICAL_AMR_TOP_PLANE_PATH
    AMR_RAW_SLOT_PATH = "/World/SF_Twin_Cell/AMR/Mockup/TraySlots/RawSlot"

    def __init__(
        self,
        stage,
        *,
        world_to_base=(0.0, 0.0, 0.0, 0.0),
        registry=None,
        cube_factory=None,
        delete_prim=None,
        rigid_body_api_factory=None,
        physx_rigid_body_api_factory=None,
    ):
        self.stage = stage
        self.transform = self.base_link_world_transform(world_to_base)
        self.registry = registry or self._new_registry()
        self.cube_factory = cube_factory
        self._delete_prim = delete_prim
        self._rigid_body_api_factory = (
            rigid_body_api_factory or self._default_rigid_body_api
        )
        self._physx_rigid_body_api_factory = (
            physx_rigid_body_api_factory or self._default_physx_rigid_body_api
        )
        self._owned_path = None
        self.pre_capture_stability = None
        self._authored_world_pose = None
        self._transform_diagnostic_emitted = False

    @staticmethod
    def base_link_world_transform(world_to_base):
        return RigidTransform.from_world_to_base(world_to_base)

    @staticmethod
    def _new_registry():
        from gripper_runtime.grasp_attachment import EligibleObjectRegistry

        return EligibleObjectRegistry()

    def spawn_cube(self, prim_path, pose, dimensions):
        prim_path = str(prim_path)
        if self._owned_path is not None:
            raise RuntimeError("validation runtime already owns one fixture cube")
        existing = self.stage.GetPrimAtPath(prim_path)
        if existing.IsValid():
            self._clear_stale_owned_fixture(prim_path, existing)
        # Validation profile geometry is explicitly world-frame.  Applying
        # base_to_world here would double-transform the seeded fixture pose.
        world_pose = pose
        if self.cube_factory is None:
            self._default_cube(prim_path, world_pose, dimensions)
        else:
            self.cube_factory(prim_path, world_pose, dimensions)
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError("Isaac cube factory did not create the fixture prim")
        self._set_owner(prim)
        self._owned_path = prim_path
        self._authored_world_pose = world_pose
        self._transform_diagnostic_emitted = False
        self.pre_capture_stability = PreCaptureStabilityGuard(
            self.stage, prim_path, self._rigid_body_api_factory
        )
        if not self.pre_capture_stability.hold_for_registration():
            delete = self._delete_prim or self._default_delete_prim
            delete(prim_path)
            self._owned_path = None
            self.pre_capture_stability = None
            raise RuntimeError("failed to establish pre-capture fixture stability")

    def _clear_stale_owned_fixture(self, prim_path, prim):
        """Remove only an un-attached, validation-owned stale fixture.

        A fresh validation runtime may encounter its own previous fixture
        after a Script Editor rerun.  That exact stale case is recoverable;
        foreign prims and fixtures with an adapter-owned grasp joint remain
        fail-closed.
        """
        if not self._is_owned(prim):
            raise RuntimeError("fixture prim path is already occupied")
        joint_path = f"{prim_path}/sf_scripted_grasp_joint"
        if self.stage.GetPrimAtPath(joint_path).IsValid():
            raise RuntimeError("cannot replace fixture with an active grasp joint")

        delete = self._delete_prim or self._default_delete_prim
        try:
            self.registry.unregister(prim_path)
            delete(prim_path)
        except Exception as error:
            raise RuntimeError("failed to remove stale validation fixture") from error
        if self.stage.GetPrimAtPath(prim_path).IsValid():
            raise RuntimeError("stale validation fixture removal was incomplete")

    def _default_cube(self, prim_path, world_pose, dimensions):
        import numpy as np

        from scene_builder.primitives import dynamic_box

        colors = np.array([0.52, 0.36, 0.20])
        dynamic_box(
            self.stage,
            prim_path,
            "wu14_validation_cube",
            [world_pose.x, world_pose.y, world_pose.z],
            dimensions,
            colors,
            mass=0.45,
        )
        prim = self.stage.GetPrimAtPath(prim_path)
        xform = self._xformable(prim)
        if abs(world_pose.yaw) > 1e-12:
            _, _, _, usd_geom, _ = self._pxr_modules()
            xform.AddRotateZOp().Set(math.degrees(world_pose.yaw))

    def disable_gravity(self, prim_path):
        prim = self._owned_prim(prim_path)
        try:
            attribute = self._physx_rigid_body_api_factory(
                prim
            ).CreateDisableGravityAttr()
        except Exception as error:
            raise RuntimeError("failed to author canonical gravity policy") from error
        attribute.Set(True)

    def enable_gravity(self, prim_path):
        prim = self._owned_prim(prim_path)
        try:
            attribute = self._physx_rigid_body_api_factory(
                prim
            ).CreateDisableGravityAttr()
            attribute.Set(False)
        except Exception as error:
            raise RuntimeError("failed to enable canonical fixture gravity") from error

    def register_eligible(self, prim_path, run_id):
        del run_id
        prim = self._owned_prim(prim_path)
        self._set_owner(prim)
        self.registry.register(str(prim_path))

    def retire_fixture_for_pending_delete(self, prim_path):
        """Keep the owned prim stable while removing it from capture candidates."""
        self._owned_prim(prim_path)
        self.disable_gravity(prim_path)
        self.registry.unregister(str(prim_path))
        return True

    def read_base_pose(self, prim_path):
        prim = self._owned_prim(prim_path)
        world = self._world_pose(prim)
        return self.transform.world_to_base(world)

    def read_world_pose(self, prim_path):
        prim = self._owned_prim(prim_path)
        pose = self._world_pose(prim)
        if not self._transform_diagnostic_emitted:
            self._emit_transform_diagnostic(prim, pose)
            self._transform_diagnostic_emitted = True
        return pose

    def resolve_amr_top_plane(self, canonical_path=None):
        """Resolve the one canonical AMR support surface and record provenance."""
        path = canonical_path or self.AMR_TOP_PLANE_PATH
        _, _, usd, usd_geom, _ = self._pxr_modules()

        def read_world_transform(prim):
            xformable = usd_geom.Xformable(prim)
            if not xformable:
                raise RuntimeError(
                    f"canonical AMR top prim is not Xformable: {prim.GetPath()}"
                )
            matrix = xformable.ComputeLocalToWorldTransform(usd.TimeCode.Default())
            if matrix is None:
                cache = usd_geom.XformCache(usd.TimeCode.Default())
                matrix = cache.GetLocalToWorldTransform(prim)
            if matrix is None:
                raise RuntimeError(
                    f"USD returned no world transform for {prim.GetPath()}"
                )
            return matrix

        identity = AmrTopPlaneResolver(path).resolve(
            self.stage,
            world_transform_reader=read_world_transform,
        )
        prim = self.stage.GetPrimAtPath(identity.prim_path)
        role = prim.GetCustomDataByKey("sf_twin:role")
        if role != "direct_pick_tray":
            raise RuntimeError(
                f"canonical AMR top-plane role mismatch at {identity.prim_path}: {role!r}"
            )
        xform = usd_geom.Xformable(prim)
        local_matrix = xform.GetLocalTransformation()
        world_matrix = identity.world_transform
        cube = usd_geom.Cube(prim)
        cube_size = cube.GetSizeAttr().Get() or 1.0

        local_geometry = cube_geometry_from_transform(
            self._matrix_rows_for_geometry(local_matrix), float(cube_size)
        )
        world_geometry = cube_geometry_from_transform(
            self._matrix_rows_for_geometry(world_matrix), float(cube_size)
        )
        local_dimensions = tuple(
            float(value) for value in local_geometry["dimensions_m"]
        )
        world_dimensions = tuple(
            float(value) for value in world_geometry["dimensions_m"]
        )
        local_scale = tuple(value / float(cube_size) for value in local_dimensions)
        # Candidate coordinates are in the Cube's intrinsic local coordinates.
        # The scale is applied only by the one LocalToWorld transform later.
        top_surface_local_z = float(cube_size) / 2.0
        parent_path = path.rsplit("/", 1)[0]
        rail_paths = [
            f"{parent_path}/TrayRails/LeftRail",
            f"{parent_path}/TrayRails/RightRail",
            f"{parent_path}/TrayRails/FrontRail",
            f"{parent_path}/TrayRails/RearRail",
        ]
        # The four rail paths are canonical scene geometry, not a fuzzy search.
        tray_inverse = world_matrix.GetInverse()
        rail_bounds = []
        rail_provenance = []
        for rail_path in rail_paths:
            rail = self.stage.GetPrimAtPath(rail_path)
            if rail is None or not rail.IsValid():
                raise RuntimeError(f"missing canonical AMR rail prim: {rail_path}")
            rail_xform = usd_geom.Xformable(rail)
            rail_matrix = rail_xform.ComputeLocalToWorldTransform(
                usd.TimeCode.Default()
            )
            relative = tray_inverse * rail_matrix
            rail_cube = usd_geom.Cube(rail)
            rail_size = rail_cube.GetSizeAttr().Get() or 1.0
            rail_geometry = cube_geometry_from_transform(
                self._matrix_rows_for_geometry(relative), float(rail_size)
            )
            rail_world_geometry = cube_geometry_from_transform(
                self._matrix_rows_for_geometry(rail_matrix), float(rail_size)
            )
            center = rail_geometry["center"]
            rail_bounds.append(
                (
                    float(center[0]),
                    float(center[1]),
                    tuple(rail_geometry["dimensions_m"]),
                )
            )
            rail_provenance.append(
                {
                    "prim_path": rail_path,
                    "center_local_m": center,
                    "center_world_m": rail_world_geometry["center"],
                    "dimensions_world_m": rail_world_geometry["dimensions_m"],
                }
            )
        x_min, x_max = -float(cube_size) / 2.0, float(cube_size) / 2.0
        y_min, y_max = -float(cube_size) / 2.0, float(cube_size) / 2.0
        for center_x, center_y, dimensions in rail_bounds:
            half_x, half_y = dimensions[0] / 2.0, dimensions[1] / 2.0
            if dimensions[0] >= dimensions[1]:
                if center_y < 0.0:
                    y_min = max(y_min, center_y + half_y)
                else:
                    y_max = min(y_max, center_y - half_y)
            else:
                if center_x < 0.0:
                    x_min = max(x_min, center_x + half_x)
                else:
                    x_max = min(x_max, center_x - half_x)
        if not (x_min < x_max and y_min < y_max):
            raise RuntimeError("AMR top-plane usable bounds are empty")
        return {
            "prim_path": identity.prim_path,
            "role": role,
            "world_transform": self._matrix_rows(world_matrix),
            "world_center_m": world_geometry["center"],
            "local_dimensions_m": list(local_dimensions),
            "local_cube_dimensions": [float(cube_size)] * 3,
            "local_scale": list(local_scale),
            "world_dimensions_m": list(world_dimensions),
            "world_aabb_min_m": world_geometry["aabb_min"],
            "world_aabb_max_m": world_geometry["aabb_max"],
            "world_top_z_m": world_geometry["top_z_m"],
            "top_surface_local_z_m": top_surface_local_z,
            "usable_bounds_local": {
                "x_min": x_min,
                "x_max": x_max,
                "y_min": y_min,
                "y_max": y_max,
            },
            "usable_dimensions_m": [x_max - x_min, y_max - y_min],
            "rail_provenance": rail_provenance,
        }

    def read_raw_slot_geometry(self, canonical_path=None):
        """Read the exact RawSlot primitive for candidate-source provenance."""
        path = canonical_path or self.AMR_RAW_SLOT_PATH
        prim = self.stage.GetPrimAtPath(path)
        if prim is None or not prim.IsValid():
            raise RuntimeError(f"missing canonical AMR RawSlot prim: {path}")
        _, _, usd, usd_geom, _ = self._pxr_modules()
        cube = usd_geom.Cube(prim)
        size = cube.GetSizeAttr().Get() or 1.0
        matrix = usd_geom.Xformable(prim).ComputeLocalToWorldTransform(
            usd.TimeCode.Default()
        )
        geometry = cube_geometry_from_transform(
            self._matrix_rows_for_geometry(matrix), float(size)
        )
        return {
            "prim_path": path,
            "world_transform": self._matrix_rows(matrix),
            "world_center_m": geometry["center"],
            "world_dimensions_m": geometry["dimensions_m"],
            "world_aabb_min_m": geometry["aabb_min"],
            "world_aabb_max_m": geometry["aabb_max"],
            "world_top_z_m": geometry["top_z_m"],
        }

    @staticmethod
    def assert_amr_top_plane_snapshot(
        support,
        *,
        expected_center=(0.20, -0.85, 0.64),
        expected_dimensions=(0.66, 0.46, 0.045),
        expected_top_z=0.6625,
        tolerance=1e-6,
    ):
        observed_center = support["world_center_m"]
        observed_dimensions = support["world_dimensions_m"]
        if any(
            not math.isclose(float(actual), float(expected), abs_tol=tolerance)
            for actual, expected in zip(observed_center, expected_center)
        ):
            raise RuntimeError("AMR Tray world center provenance mismatch")
        if any(
            not math.isclose(float(actual), float(expected), abs_tol=tolerance)
            for actual, expected in zip(observed_dimensions, expected_dimensions)
        ):
            raise RuntimeError("AMR Tray world dimensions provenance mismatch")
        if not math.isclose(
            float(support["world_top_z_m"]), float(expected_top_z), abs_tol=tolerance
        ):
            raise RuntimeError("AMR Tray top-surface provenance mismatch")
        return True

    @staticmethod
    def _matrix_rows(matrix):
        return [[float(matrix[row][column]) for column in range(4)] for row in range(4)]

    @classmethod
    def _matrix_rows_for_geometry(cls, matrix):
        """Convert USD/Gf row-vector storage to the pure helper's column form."""
        rows = cls._matrix_rows(matrix)
        return [[rows[column][row] for column in range(4)] for row in range(4)]

    def amr_candidate_to_world(
        self,
        candidate,
        *,
        fixture_dimensions,
        clearance_m=0.001,
        canonical_path=None,
    ):
        """Convert one AMR-local candidate to one world-frame cube pose."""
        support = self.resolve_amr_top_plane(canonical_path)
        prim = self.stage.GetPrimAtPath(support["prim_path"])
        _, _, _, usd_geom, _ = self._pxr_modules()
        matrix = usd_geom.Xformable(prim).ComputeLocalToWorldTransform(
            self._pxr_modules()[2].TimeCode.Default()
        )
        pose = candidate["pose"]
        local_z = float(pose["z"])
        gf = self._pxr_modules()[0]
        point = matrix.Transform(gf.Vec3d(float(pose["x"]), float(pose["y"]), local_z))
        parent_yaw = self._yaw_from_components(
            self._rotation_quaternion_from_matrix(matrix)
        )
        yaw = math.atan2(
            math.sin(parent_yaw + float(pose["yaw"])),
            math.cos(parent_yaw + float(pose["yaw"])),
        )
        return {
            "candidate_id": candidate["candidate_id"],
            "support_surface_id": support["prim_path"],
            "support_surface_frame": "amr_top",
            "sampled_pose_amr_top": dict(pose),
            "spawn_pose_world": {
                "frame_id": "world",
                "x": float(point[0]),
                "y": float(point[1]),
                "z": float(point[2]),
                "yaw": yaw,
            },
            "fixture_dimensions_m": list(fixture_dimensions),
            "spawn_clearance_m": float(clearance_m),
            "support_surface_world_transform": support["world_transform"],
        }

    def spawn_from_amr(self, prim_path, profile):
        """Spawn the seeded fixture on the resolved AMR top plane."""
        support = self.resolve_amr_top_plane(profile.support_surface_id)
        local_scale = tuple(float(value) for value in support["local_scale"])
        local_fixture_dimensions = tuple(
            float(value) / scale
            for value, scale in zip(profile.fixture_dimensions, local_scale)
        )
        local_clearance = profile.spawn_clearance_m / local_scale[2]
        candidate = sample_amr_top_candidate(
            seed=profile.seed,
            index=0,
            usable_dimensions=(
                float(support["usable_dimensions_m"][0]),
                float(support["usable_dimensions_m"][1]),
            ),
            usable_bounds_local=support["usable_bounds_local"],
            fixture_dimensions=profile.fixture_dimensions,
            sampling_fixture_dimensions=local_fixture_dimensions,
            top_surface_z=float(support["top_surface_local_z_m"]),
            clearance_m=local_clearance,
        )
        provenance = self.amr_candidate_to_world(
            candidate,
            fixture_dimensions=profile.fixture_dimensions,
            clearance_m=profile.spawn_clearance_m,
            canonical_path=profile.support_surface_id,
        )
        spawn = provenance["spawn_pose_world"]
        pose = Pose(spawn["x"], spawn["y"], spawn["z"], spawn["yaw"])
        self.spawn_cube(prim_path, pose, profile.fixture_dimensions)
        return pose, provenance

    def existing_prim_paths(self):
        paths = []
        for prim in self.stage.Traverse():
            if prim.IsValid() and self._is_owned(prim):
                paths.append(str(prim.GetPath()))
        return paths

    def adopt_existing_fixture(self, prim_path):
        """Adopt only an already-owned validation prim; never fuzzy-discover one."""
        prim_path = str(prim_path)
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid() or not self._is_owned(prim):
            raise RuntimeError("cannot adopt unknown or foreign fixture prim")
        if self._owned_path is not None and self._owned_path != prim_path:
            raise RuntimeError("validation runtime already owns another fixture")
        self._owned_path = prim_path
        self.pre_capture_stability = self._new_stability_guard(prim_path)
        return prim_path

    def delete_prim(self, prim_path):
        prim_path = str(prim_path)
        if self._owned_path != prim_path:
            return False
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid() or not self._is_owned(prim):
            return False
        self.registry.unregister(prim_path)
        delete = self._delete_prim or self._default_delete_prim
        delete(prim_path)
        if self.stage.GetPrimAtPath(prim_path).IsValid():
            return False
        self._owned_path = None
        self.pre_capture_stability = None
        return True

    def _owned_prim(self, prim_path):
        prim_path = str(prim_path)
        if self._owned_path != prim_path:
            raise RuntimeError("prim is not owned by this validation fixture")
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid() or not self._is_owned(prim):
            raise RuntimeError("fixture prim identity or ownership marker is invalid")
        return prim

    def _new_stability_guard(self, prim_path):
        return PreCaptureStabilityGuard(
            self.stage, prim_path, self._rigid_body_api_factory
        )

    @classmethod
    def _set_owner(cls, prim):
        prim.SetCustomDataByKey(cls.OWNER_KEY, cls.OWNER_VALUE)

    @classmethod
    def _is_owned(cls, prim):
        return prim.GetCustomDataByKey(cls.OWNER_KEY) == cls.OWNER_VALUE

    @staticmethod
    def _pxr_modules():
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

        return Gf, Sdf, Usd, UsdGeom, UsdPhysics

    @classmethod
    def _xformable(cls, prim):
        return cls._pxr_modules()[3].Xformable(prim)

    @classmethod
    def _world_pose(cls, prim):
        _, _, usd, usd_geom, _ = cls._pxr_modules()
        matrix = usd_geom.Xformable(prim).ComputeLocalToWorldTransform(
            usd.TimeCode.Default()
        )
        translation = matrix.ExtractTranslation()
        quaternion = cls._rotation_quaternion_from_matrix(matrix)
        yaw = cls._yaw_from_components(quaternion)
        return Pose(
            float(translation[0]), float(translation[1]), float(translation[2]), yaw
        )

    def _emit_transform_diagnostic(self, prim, observed_pose):
        """Print one read-only transform provenance record for live diagnosis."""
        try:
            _, _, usd, usd_geom, _ = self._pxr_modules()
            xform = usd_geom.Xformable(prim)
            matrix = xform.ComputeLocalToWorldTransform(usd.TimeCode.Default())
            raw_quaternion = matrix.ExtractRotationQuat()
            basis = self._orthonormalized_rotation_basis(matrix)
            basis_quaternion = self._quaternion_from_rotation_basis(basis)
            transposed_basis = self._transpose_basis(basis)
            quaternion = self._quaternion_from_rotation_basis(transposed_basis)
            basis_yaw = self._yaw_from_components(basis_quaternion)
            transposed_basis_yaw = self._yaw_from_components(quaternion)
            roll, pitch, yaw = self._rpy_from_components(quaternion)
            authored = self._authored_world_pose
            if authored is None:
                authored = Pose(
                    float(matrix.ExtractTranslation()[0]),
                    float(matrix.ExtractTranslation()[1]),
                    float(matrix.ExtractTranslation()[2]),
                    yaw,
                )
            print("[WU-14] fixture transform diagnostic begin")
            print(
                "[WU-14] authored_world_pose "
                f"x={authored.x:.15f} y={authored.y:.15f} "
                f"z={authored.z:.15f} yaw={authored.yaw:.15f}"
            )
            print("[WU-14] local_xform_ops:")
            for index, op in enumerate(xform.GetOrderedXformOps()):
                print(
                    f"[WU-14]   [{index}] type={op.GetOpType()} "
                    f"name={op.GetOpName()} value={op.Get()}"
                )
            print(f"[WU-14] local_to_world_matrix={matrix}")
            print(f"[WU-14] orthonormalized_basis={basis}")
            print(
                "[WU-14] basis_quaternion "
                f"w={basis_quaternion[0]:.15f} x={basis_quaternion[1]:.15f} "
                f"y={basis_quaternion[2]:.15f} z={basis_quaternion[3]:.15f} "
                f"yaw={basis_yaw:.15f}"
            )
            print(f"[WU-14] transposed_basis={transposed_basis}")
            print(
                "[WU-14] transposed_basis_quaternion "
                f"w={quaternion[0]:.15f} x={quaternion[1]:.15f} "
                f"y={quaternion[2]:.15f} z={quaternion[3]:.15f} "
                f"yaw={transposed_basis_yaw:.15f}"
            )
            print(
                "[WU-14] raw_world_quaternion "
                f"real_w={raw_quaternion.GetReal()} "
                f"imaginary_xyz={raw_quaternion.GetImaginary()}"
            )
            print(
                "[WU-14] normalized_world_quaternion "
                f"w={quaternion[0]:.15f} x={quaternion[1]:.15f} "
                f"y={quaternion[2]:.15f} z={quaternion[3]:.15f} "
                f"norm={self._quaternion_norm(quaternion):.15f}"
            )
            print(
                "[WU-14] world_rpy "
                f"roll={roll:.15f} pitch={pitch:.15f} yaw={yaw:.15f}"
            )
            print(
                "[WU-14] expected_vs_actual "
                f"expected_yaw={authored.yaw:.15f} "
                f"actual_yaw={observed_pose.yaw:.15f} "
                f"angular_delta={_angular_distance(authored.yaw, observed_pose.yaw):.15f}"
            )
            parent = prim.GetParent()
            if parent is not None and parent.IsValid():
                parent_matrix = usd_geom.Xformable(parent).ComputeLocalToWorldTransform(
                    usd.TimeCode.Default()
                )
                parent_quaternion = self._rotation_quaternion_from_matrix(parent_matrix)
                parent_roll, parent_pitch, parent_yaw = self._rpy_from_components(
                    parent_quaternion
                )
                print(f"[WU-14] parent_path={parent.GetPath()}")
                print(f"[WU-14] parent_local_to_world_matrix={parent_matrix}")
                print(
                    "[WU-14] parent_world_rpy "
                    f"roll={parent_roll:.15f} pitch={parent_pitch:.15f} "
                    f"yaw={parent_yaw:.15f}"
                )
                print(
                    "[WU-14] parent_normalized_quaternion "
                    f"w={parent_quaternion[0]:.15f} "
                    f"x={parent_quaternion[1]:.15f} "
                    f"y={parent_quaternion[2]:.15f} "
                    f"z={parent_quaternion[3]:.15f} "
                    f"norm={self._quaternion_norm(parent_quaternion):.15f}"
                )
            else:
                print("[WU-14] parent_path=<none>")
            print("[WU-14] fixture transform diagnostic end")
        except Exception as error:
            print(f"[WU-14] fixture transform diagnostic unavailable: {error}")

    @staticmethod
    def _quaternion_norm(quaternion):
        return math.sqrt(sum(float(value) ** 2 for value in quaternion))

    @classmethod
    def _rotation_quaternion_from_matrix(cls, matrix):
        """Extract USD/Gf rotation after removing scale and transposing basis."""
        basis = cls._orthonormalized_rotation_basis(matrix)
        # Gf/USD's authored rotation basis is represented with the opposite
        # row/column convention from the quaternion conversion below.  The
        # transpose is the convention conversion, not a sign correction.
        return cls._quaternion_from_rotation_basis(cls._transpose_basis(basis))

    @staticmethod
    def _transpose_basis(basis):
        return tuple(
            tuple(basis[row][column] for row in range(3)) for column in range(3)
        )

    @classmethod
    def _orthonormalized_rotation_basis(cls, matrix):
        """Return the scale-free USD/Gf 3x3 basis before convention conversion."""
        columns = [
            [float(matrix[row][column]) for row in range(3)] for column in range(3)
        ]

        def dot(first, second):
            return sum(left * right for left, right in zip(first, second))

        def cross(first, second):
            return [
                first[1] * second[2] - first[2] * second[1],
                first[2] * second[0] - first[0] * second[2],
                first[0] * second[1] - first[1] * second[0],
            ]

        def normalize(vector, label):
            norm = math.sqrt(dot(vector, vector))
            if not math.isfinite(norm) or norm <= 1e-12:
                raise RuntimeError(f"cannot normalize {label} rotation basis")
            return [value / norm for value in vector]

        x_axis = normalize(columns[0], "x")
        y_axis = [
            value - dot(columns[1], x_axis) * axis
            for value, axis in zip(columns[1], x_axis)
        ]
        y_axis = normalize(y_axis, "y")
        z_axis = cross(x_axis, y_axis)
        if dot(z_axis, columns[2]) < 0.0:
            z_axis = [-value for value in z_axis]
        y_axis = normalize(cross(z_axis, x_axis), "y")
        rotation = (
            (x_axis[0], y_axis[0], z_axis[0]),
            (x_axis[1], y_axis[1], z_axis[1]),
            (x_axis[2], y_axis[2], z_axis[2]),
        )
        return rotation

    @classmethod
    def _quaternion_from_rotation_basis(cls, rotation):
        trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            quaternion = (
                0.25 * scale,
                (rotation[2][1] - rotation[1][2]) / scale,
                (rotation[0][2] - rotation[2][0]) / scale,
                (rotation[1][0] - rotation[0][1]) / scale,
            )
        elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
            scale = (
                math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
            )
            quaternion = (
                (rotation[2][1] - rotation[1][2]) / scale,
                0.25 * scale,
                (rotation[0][1] + rotation[1][0]) / scale,
                (rotation[0][2] + rotation[2][0]) / scale,
            )
        elif rotation[1][1] > rotation[2][2]:
            scale = (
                math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
            )
            quaternion = (
                (rotation[0][2] - rotation[2][0]) / scale,
                (rotation[0][1] + rotation[1][0]) / scale,
                0.25 * scale,
                (rotation[1][2] + rotation[2][1]) / scale,
            )
        else:
            scale = (
                math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
            )
            quaternion = (
                (rotation[1][0] - rotation[0][1]) / scale,
                (rotation[0][2] + rotation[2][0]) / scale,
                (rotation[1][2] + rotation[2][1]) / scale,
                0.25 * scale,
            )
        norm = cls._quaternion_norm(quaternion)
        if not math.isfinite(norm) or abs(norm - 1.0) > 1e-6:
            raise RuntimeError("fixture rotation quaternion is not unit-normalized")
        return tuple(value / norm for value in quaternion)

    @staticmethod
    def _rpy_from_components(quaternion):
        w, x, y, z = quaternion
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_argument = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, pitch_argument)))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    @classmethod
    def _rpy_from_quaternion(cls, quaternion):
        components = (
            float(quaternion.GetReal()),
            *(float(value) for value in quaternion.GetImaginary()),
        )
        norm = cls._quaternion_norm(components)
        if not math.isfinite(norm) or norm <= 1e-12:
            raise RuntimeError("quaternion is not finite and non-zero")
        return cls._rpy_from_components(tuple(value / norm for value in components))

    @staticmethod
    def _yaw_from_components(quaternion):
        return IsaacFixtureRuntime._rpy_from_components(quaternion)[2]

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        """Extract Z yaw from a pxr quaternion (real=w, imaginary=(x,y,z))."""
        w = float(quaternion.GetReal())
        x, y, z = (float(value) for value in quaternion.GetImaginary())
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @staticmethod
    def _default_rigid_body_api(prim):
        from pxr import UsdPhysics

        api = UsdPhysics.RigidBodyAPI(prim)
        if not api:
            api = UsdPhysics.RigidBodyAPI.Apply(prim)
        return api

    @staticmethod
    def _default_physx_rigid_body_api(prim):
        from pxr import PhysxSchema

        api = PhysxSchema.PhysxRigidBodyAPI(prim)
        if not api:
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        return api

    @staticmethod
    def _bool_type():
        try:
            from pxr import Sdf
        except ImportError:
            return bool
        return Sdf.ValueTypeNames.Bool

    @staticmethod
    def _default_delete_prim(prim_path):
        try:
            from isaacsim.core.utils.prims import delete_prim
        except ImportError:
            from omni.isaac.core.utils.prims import delete_prim

        delete_prim(prim_path)


class IsaacFixtureCommandWorker:
    """Validation-private Isaac update-loop worker for external fixture control."""

    def __init__(
        self,
        runtime,
        *,
        command_path,
        response_path,
        run_id,
        fixture_id,
        fixture_dimensions,
        seed,
        clearance_m=0.001,
    ):
        self.runtime = runtime
        self.fixture_dimensions = tuple(float(value) for value in fixture_dimensions)
        self.seed = int(seed)
        self.clearance_m = float(clearance_m)
        self.consumer = FixtureRuntimeConsumer(
            command_path,
            response_path,
            run_id=run_id,
            fixture_id=fixture_id,
            handlers={
                "spawn_fixture": self._spawn,
                "remove_fixture": self._remove,
                "query_fixture": self._query,
                "respawn_fixture": self._respawn,
            },
        )

    def poll(self):
        return self.consumer.poll()

    def _candidate(self, payload):
        seed = int(payload.get("seed", self.seed))
        index = int(payload.get("index", 0))
        support = self.runtime.resolve_amr_top_plane()
        local_scale = tuple(float(value) for value in support["local_scale"])
        local_fixture_dimensions = tuple(
            float(value) / scale
            for value, scale in zip(self.fixture_dimensions, local_scale)
        )
        candidate = sample_amr_top_candidate(
            seed=seed,
            index=index,
            usable_dimensions=(
                float(support["usable_dimensions_m"][0]),
                float(support["usable_dimensions_m"][1]),
            ),
            usable_bounds_local=support["usable_bounds_local"],
            fixture_dimensions=self.fixture_dimensions,
            sampling_fixture_dimensions=local_fixture_dimensions,
            top_surface_z=float(support["top_surface_local_z_m"]),
            clearance_m=self.clearance_m / local_scale[2],
        )
        expected_id = payload.get("candidate_id")
        if expected_id is not None and expected_id != candidate["candidate_id"]:
            raise RuntimeError("candidate identity mismatch")
        return candidate

    def _spawn(self, payload):
        candidate = self._candidate(payload)
        provenance = self.runtime.amr_candidate_to_world(
            candidate,
            fixture_dimensions=self.fixture_dimensions,
            clearance_m=self.clearance_m,
        )
        spawn = provenance["spawn_pose_world"]
        prim_path = str(
            payload.get("prim_path", "/World/SFTwin/ValidationFixture/Cube")
        )
        self.runtime.spawn_cube(
            prim_path,
            Pose(spawn["x"], spawn["y"], spawn["z"], spawn["yaw"]),
            self.fixture_dimensions,
        )
        self.runtime.disable_gravity(prim_path)
        actual_world = self.runtime.read_world_pose(prim_path)
        actual_base = self.runtime.read_base_pose(prim_path)
        provenance["actual_pose_world"] = {
            "frame_id": "world",
            "x": actual_world.x,
            "y": actual_world.y,
            "z": actual_world.z,
            "yaw": actual_world.yaw,
        }
        provenance["actual_pose_base_link"] = {
            "frame_id": "base_link",
            "x": actual_base.x,
            "y": actual_base.y,
            "z": actual_base.z,
            "yaw": actual_base.yaw,
        }
        return provenance

    def _remove(self, payload):
        prim_path = str(
            payload.get("prim_path", "/World/SFTwin/ValidationFixture/Cube")
        )
        if not self.runtime.delete_prim(prim_path):
            raise RuntimeError("fixture removal was not ownership-confirmed")
        return {"prim_path": prim_path, "removed": True}

    def _query(self, payload):
        prim_path = str(
            payload.get("prim_path", "/World/SFTwin/ValidationFixture/Cube")
        )
        support = self.runtime.resolve_amr_top_plane()
        raw_slot = self.runtime.read_raw_slot_geometry()
        geometry = {
            "support_geometry": support,
            "raw_slot_geometry": raw_slot,
        }
        if self.runtime._owned_path != prim_path:
            return {"prim_path": prim_path, "owned": False, **geometry}
        world = self.runtime.read_world_pose(prim_path)
        return {
            "prim_path": prim_path,
            "owned": True,
            "pose_world": {"x": world.x, "y": world.y, "z": world.z, "yaw": world.yaw},
            **geometry,
        }

    def _respawn(self, payload):
        prim_path = str(
            payload.get("prim_path", "/World/SFTwin/ValidationFixture/Cube")
        )
        if self.runtime._owned_path == prim_path:
            self._remove({"prim_path": prim_path})
        return self._spawn(payload)


@dataclass
class ValidationRun:
    profile: FixtureProfile
    runtime: FixtureRuntime
    run_id: str
    prim_path: str = "/World/SFTwin/ValidationFixture/Cube"
    clock_s: Callable[[], float] = lambda: 0.0
    ownership: FixtureOwnership = None  # type: ignore[assignment]
    manifest: ValidationManifest | None = None
    cleanup: CleanupDecision | None = None
    cleanup_decision_id: str | None = None
    cleanup_confirmation_path: Path | None = None
    physical_delete_state: str = "active"
    _pending_delete_request: dict | None = None
    _last_cleanup_log_key: tuple[bool, str] | None = None
    gravity_activation_attempted: bool = False
    gravity_activation_succeeded: bool | None = None
    post_release_observation_pending: bool = False
    _post_release_observation_ticks: int = 0
    _post_release_expected_position: tuple[float, float, float] | None = None

    def __post_init__(self):
        self.ownership = self.ownership or FixtureOwnership()

    def start(self) -> ValidationManifest:
        """Spawn one cube, wire it to Fixed Vision, and return debug manifest."""
        if self.manifest is not None:
            raise RuntimeError("validation run has already started")
        provenance = None
        spawn_from_amr = getattr(self.runtime, "spawn_from_amr", None)
        if self.profile.support_surface_id and callable(spawn_from_amr):
            spawn_pose_world, provenance = spawn_from_amr(self.prim_path, self.profile)
        else:
            spawn_pose_world = sample_spawn_pose(self.profile)
            self.runtime.spawn_cube(
                self.prim_path, spawn_pose_world, self.profile.fixture_dimensions
            )
        self.ownership.claim(self.prim_path)
        try:
            self.runtime.disable_gravity(self.prim_path)
            self.runtime.register_eligible(self.prim_path, self.run_id)
            observed_world_pose = getattr(self.runtime, "read_world_pose", None)
            if not callable(observed_world_pose):
                raise RuntimeError("fixture runtime cannot read world pose")
            observed_world = observed_world_pose(self.prim_path)
            if (
                _angular_distance(observed_world.yaw, spawn_pose_world.yaw)
                > self.profile.orientation_tolerance_rad
            ):
                raise RuntimeError(
                    "fixture world yaw readback exceeds validation tolerance"
                )
            spawn_pose_world = observed_world
            observed_base_pose = self.runtime.read_base_pose(self.prim_path)
        except Exception:
            self.abort()
            raise
        self.manifest = ValidationManifest.from_fixture(
            run_id=self.run_id,
            profile=self.profile,
            prim_path=self.prim_path,
            spawn_pose_world=spawn_pose_world,
            registered_top_surface_pose_base_link=top_surface_reference(
                observed_base_pose, self.profile.fixture_dimensions
            ),
        )
        if provenance is not None:
            self.manifest = replace(
                self.manifest,
                sampled_pose_amr_top=provenance["sampled_pose_amr_top"],
                support_surface_world_transform=provenance[
                    "support_surface_world_transform"
                ],
            )
        return self.manifest

    def on_place_result(
        self,
        *,
        success: bool,
        fresh_released: bool,
        detach_confirmed: bool,
        ownership_unambiguous: bool,
        actual_pose: Pose | None,
        expected_position_world: tuple[float, float, float],
        confirmation_path: str | Path | None = None,
    ) -> CleanupDecision:
        """Record accepted release, activate gravity, then observe the fixture."""
        if (
            success
            and fresh_released
            and detach_confirmed
            and ownership_unambiguous
            and self.ownership.owns(self.prim_path)
            and not self.gravity_activation_attempted
        ):
            self.gravity_activation_attempted = True
            try:
                self.runtime.enable_gravity(self.prim_path)
            except Exception as error:
                self.gravity_activation_succeeded = False
                print(
                    "[WU-14] place_motion_result=SUCCESS "
                    "gravity_activation=FAILED post-release diagnostic: "
                    f"fixture={self.prim_path} error={error}"
                )
            else:
                self.gravity_activation_succeeded = True
        if success and fresh_released and detach_confirmed and ownership_unambiguous:
            self.post_release_observation_pending = True
            self._post_release_observation_ticks = 1
            self._post_release_expected_position = expected_position_world
        self.cleanup = cleanup_decision(
            self.ownership,
            prim_path=self.prim_path,
            place_succeeded=success,
            fresh_released=fresh_released,
            detach_confirmed=detach_confirmed,
            ownership_unambiguous=ownership_unambiguous,
            actual_pose=actual_pose,
            expected_position=expected_position_world,
            position_tolerance_m=self.profile.place_position_tolerance_m,
            now_s=self.clock_s(),
            delay_s=self.profile.deletion_delay_s,
        )
        self.cleanup_decision_id = None
        log_key = (self.cleanup.eligible, self.cleanup.reason)
        if log_key != self._last_cleanup_log_key:
            self._last_cleanup_log_key = log_key
            expected = self.cleanup.expected_position
            actual = self.cleanup.actual_position
            place_result = (
                "SUCCESS"
                if success and fresh_released and detach_confirmed
                else "FAILED"
            )
            print(
                f"[WU-14] place_motion_result={place_result} "
                "post_release_position_observation="
                f"{self.cleanup.post_release_position_observation} "
                f"expected_fixture_release_world={expected} "
                f"actual_fixture_world={actual} "
                f"position_error_m={self.cleanup.position_error_m} "
                f"tolerance_m={self.cleanup.position_tolerance_m} "
                f"eligible={self.cleanup.eligible} reason={self.cleanup.reason}"
            )
        path = confirmation_path or self.cleanup_confirmation_path
        if self.cleanup.eligible and path is not None:
            self.cleanup_decision_id = self._make_cleanup_decision_id()
            atomic_write_json(path, self._cleanup_confirmation())
        return self.cleanup

    def observe_post_release_position(self) -> str | None:
        """Sample one post-gravity fixture pose after simulation advances."""
        if not self.post_release_observation_pending:
            return None
        if self._post_release_observation_ticks > 0:
            self._post_release_observation_ticks -= 1
            return None
        self.post_release_observation_pending = False
        try:
            pose = self.runtime.read_world_pose(self.prim_path)
        except Exception:
            pose = None
        observation = cleanup_decision(
            self.ownership,
            prim_path=self.prim_path,
            place_succeeded=True,
            fresh_released=True,
            detach_confirmed=True,
            ownership_unambiguous=True,
            actual_pose=pose,
            expected_position=self._post_release_expected_position or (),
            position_tolerance_m=self.profile.place_position_tolerance_m,
            now_s=self.clock_s(),
            delay_s=self.profile.deletion_delay_s,
        )
        self.cleanup = replace(
            self.cleanup,
            expected_position=observation.expected_position,
            actual_position=observation.actual_position,
            position_error_m=observation.position_error_m,
            position_tolerance_m=observation.position_tolerance_m,
            post_release_position_observation=observation.post_release_position_observation,
        )
        print(
            "[WU-14] post_release_position_observation="
            f"{self.cleanup.post_release_position_observation} "
            f"actual_fixture_world={self.cleanup.actual_position} "
            f"position_error_m={self.cleanup.position_error_m} "
            f"tolerance_m={self.cleanup.position_tolerance_m}"
        )
        return self.cleanup.post_release_position_observation

    def _make_cleanup_decision_id(self) -> str:
        canonical = json.dumps(
            {
                "run_id": self.run_id,
                "target_id": self.profile.target_id,
                "fixture_prim_identity": self.prim_path,
                "deadline_s": self.cleanup.deadline_s,
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()

    def _cleanup_confirmation(self) -> dict[str, object]:
        position = self.cleanup.actual_position
        return {
            "schema_version": CLEANUP_HANDOFF_SCHEMA_VERSION,
            "status": "AUTHORIZED",
            "run_id": self.run_id,
            "target_id": self.profile.target_id,
            "fixture_prim_identity": self.prim_path,
            "decision_id": self.cleanup_decision_id,
            "cleanup_deadline_s": self.cleanup.deadline_s,
            "fresh_released": True,
            "detach_confirmed": True,
            "ownership_unambiguous": True,
            "post_release_position_observation": (
                self.cleanup.post_release_position_observation
            ),
            "pose": (
                None
                if position is None
                else {"x": position[0], "y": position[1], "z": position[2]}
            ),
        }

    def process_delete_request(
        self, request: dict, ack_path: str | Path, *, now_s: float | None = None
    ) -> bool:
        """Retire a semantically removed fixture; defer USD destruction."""
        decision_id = request.get("decision_id")
        if (
            self._pending_delete_request is not None
            and self._pending_delete_request.get("decision_id") == decision_id
            and self.physical_delete_state == "pending_physical_delete"
            and self._matches_cleanup_context(request)
        ):
            return True
        if (
            self._pending_delete_request is not None
            and self._pending_delete_request.get("decision_id") == decision_id
            and self.physical_delete_state == "physically_deleted"
            and self._matches_cleanup_context(request)
        ):
            atomic_write_json(ack_path, self._cleanup_ack(request, "DELETED"))
            return True
        if not self._matches_cleanup_context(request):
            return False
        if self.cleanup is None or not self.cleanup.is_due(
            self.clock_s() if now_s is None else now_s
        ):
            return False
        if not self.ownership.owns(self.prim_path):
            return False
        if self.physical_delete_state != "active":
            return False
        if not self.runtime.retire_fixture_for_pending_delete(self.prim_path):
            return False
        self._pending_delete_request = dict(request)
        self.physical_delete_state = "pending_physical_delete"
        print(
            "fixture physical delete deferred: "
            f"id={self.prim_path} reason=mission still active"
        )
        self.cleanup = replace(
            self.cleanup,
            eligible=False,
            deadline_s=None,
            reason="semantic cleanup complete; physical delete pending",
        )
        atomic_write_json(
            ack_path,
            self._cleanup_ack(request, "PHYSICAL_DELETE_PENDING"),
        )
        return True

    def complete_mission(self, event: dict, ack_path: str | Path) -> bool:
        """Physically delete the retired fixture after GO_HOME and depletion."""
        if self.physical_delete_state == "physically_deleted":
            return True
        request = self._pending_delete_request
        if self.physical_delete_state != "pending_physical_delete" or request is None:
            return False
        if not (
            isinstance(event, dict)
            and event.get("schema_version") == MISSION_COMPLETION_SCHEMA_VERSION
            and event.get("status") == "DEPLETED"
            and event.get("run_id") == self.run_id
            and event.get("target_id") == self.profile.target_id
            and event.get("go_home_complete") is True
            and event.get("scene_absent") is True
            and event.get("action_status") == 4
            and event.get("exit_reason") == "DEPLETED"
            and isinstance(event.get("goal_id"), str)
            and bool(event.get("goal_id"))
            and request.get("scene_absent_confirmed") is True
            and request.get("fresh_released") is True
            and request.get("detach_confirmed") is True
            and request.get("ownership_unambiguous") is True
            and self.ownership.owns(self.prim_path)
        ):
            return False
        deleted = self.runtime.delete_prim(self.prim_path)
        if deleted is False:
            return False
        self.ownership.release(self.prim_path)
        self.physical_delete_state = "physically_deleted"
        print(
            "fixture physical delete: "
            f"id={self.prim_path} trigger=mission_complete "
            "go_home_complete=true scene_absent=true"
        )
        atomic_write_json(ack_path, self._cleanup_ack(request, "DELETED"))
        return True

    def _matches_cleanup_context(self, request: dict) -> bool:
        return (
            request.get("schema_version") == CLEANUP_HANDOFF_SCHEMA_VERSION
            and request.get("status") == "DELETE_REQUESTED"
            and request.get("run_id") == self.run_id
            and request.get("target_id") == self.profile.target_id
            and request.get("fixture_prim_identity") == self.prim_path
            and request.get("decision_id") == self.cleanup_decision_id
            and request.get("fresh_released") is True
            and request.get("detach_confirmed") is True
            and request.get("ownership_unambiguous") is True
            and request.get("scene_absent_confirmed") is True
        )

    @staticmethod
    def _cleanup_ack(request: dict, status: str) -> dict[str, object]:
        return {
            "schema_version": CLEANUP_HANDOFF_SCHEMA_VERSION,
            "status": status,
            "run_id": request["run_id"],
            "target_id": request["target_id"],
            "fixture_prim_identity": request["fixture_prim_identity"],
            "decision_id": request["decision_id"],
        }

    def abort(self) -> None:
        """Delete fixture-owned material while preserving foreign prims."""
        for prim_path in self.ownership.abort_cleanup(
            self.runtime.existing_prim_paths()
        ):
            self.runtime.delete_prim(prim_path)
            self.ownership.release(prim_path)


class CleanupRequestWorker:
    """Poll the ROS request seam from Isaac's existing update loop."""

    def __init__(self, run: ValidationRun, request_path, ack_path):
        self.run = run
        self.request_path = Path(request_path)
        self.ack_path = Path(ack_path)

    def poll(self, now_s: float | None = None) -> bool:
        if not self.request_path.is_file():
            return False
        try:
            request = json.loads(self.request_path.read_text())
        except (OSError, json.JSONDecodeError):
            return False
        return self.run.process_delete_request(request, self.ack_path, now_s=now_s)


class MissionCompletionEventWorker:
    """Consume depletion/GO_HOME evidence and perform one final prim delete."""

    def __init__(self, run: ValidationRun, event_path, ack_path):
        self.run = run
        self.event_path = Path(event_path)
        self.ack_path = Path(ack_path)

    def poll(self) -> bool:
        if not self.event_path.is_file():
            return False
        try:
            event = json.loads(self.event_path.read_text())
        except (OSError, json.JSONDecodeError):
            return False
        return self.run.complete_mission(event, self.ack_path)


class PlaceResultEventWorker:
    """Consume one ROS PLACE fact and persist its idempotent consumption."""

    SCHEMA_VERSION = "wu14-pnp-place-result/v1"

    def __init__(
        self,
        run_id: str,
        target_id: str,
        event_path,
        consumed_path,
        consumer,
    ):
        self.run_id = run_id
        self.target_id = target_id
        self.event_path = Path(event_path)
        self.consumed_path = Path(consumed_path)
        self.consumer = consumer
        self._last_identity = None

    def poll(self) -> bool:
        if not self.event_path.is_file():
            return False
        try:
            event = json.loads(self.event_path.read_text())
        except (OSError, json.JSONDecodeError):
            return False
        if not self._valid_event(event):
            return False
        identity = event["result_identity"]
        if identity == self._last_identity or self._already_consumed(identity):
            return False
        if not self.consumer(event):
            return False
        atomic_write_json(
            self.consumed_path,
            {
                "schema_version": self.SCHEMA_VERSION,
                "status": "CONSUMED",
                "run_id": self.run_id,
                "target_id": self.target_id,
                "result_identity": identity,
            },
        )
        self._last_identity = identity
        return True

    def _valid_event(self, event: dict) -> bool:
        return (
            event.get("schema_version") == self.SCHEMA_VERSION
            and event.get("status") == "SUCCESS"
            and event.get("run_id") == self.run_id
            and event.get("target_id") == self.target_id
            and isinstance(event.get("result_identity"), str)
            and bool(event["result_identity"])
            and isinstance(event.get("sequence"), int)
            and event["sequence"] > 0
        )

    def _already_consumed(self, identity: str) -> bool:
        if not self.consumed_path.is_file():
            return False
        try:
            consumed = json.loads(self.consumed_path.read_text())
        except (OSError, json.JSONDecodeError):
            return False
        return (
            consumed.get("schema_version") == self.SCHEMA_VERSION
            and consumed.get("status") == "CONSUMED"
            and consumed.get("run_id") == self.run_id
            and consumed.get("target_id") == self.target_id
            and consumed.get("result_identity") == identity
        )
