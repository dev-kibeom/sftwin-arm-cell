"""Validation profile loading and conversion into fixture/capture settings."""

import json
import math
from pathlib import Path

import yaml
from gripper_runtime.grasp_policy import CaptureConfig
from gripper_runtime.grasp_stage import CaptureReferenceConfig
from gripper_runtime.robotiq_actuator import Robotiq2F85Actuator

from .fixture import (
    SUPPORTED_GEOMETRY_FRAMES,
    Aabb,
    FixtureProfile,
    fixture_center_from_tcp_target,
)

CANONICAL_AMR_TOP_PLANE_PATH = "/World/SF_Twin_Cell/AMR/Mockup/Tray"


def capture_config_from_profile(profile: dict) -> CaptureConfig:
    """Load Task 3's canonical CaptureConfig without unit conversion."""
    capture = profile.get("capture", {})
    if "orientation_tolerance_rad" in capture:
        raise ValueError(
            "validation capture profile must use orientation_tolerance_deg"
        )
    return CaptureConfig(
        capture_volume_dimensions_m=tuple(capture["volume_dimensions_m"]),
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
    recipe = json.loads(recipe_path.read_text())
    # PICK Approach belongs to the selected mission Recipe, not Motion tuning.
    pick = recipe.get("pick") if isinstance(recipe, dict) else None
    approach_distance = (
        pick.get("approach_distance_m") if isinstance(pick, dict) else None
    )
    if (
        isinstance(approach_distance, bool)
        or not isinstance(approach_distance, (int, float))
        or not math.isclose(approach_distance, 0.05, abs_tol=1e-9)
    ):
        raise ValueError(
            "validation recipe pick.approach_distance_m must remain 0.05 m"
        )
    profile["_mission_recipe"] = recipe
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
            float(config["place"]["orientation_tolerance_deg"])
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
