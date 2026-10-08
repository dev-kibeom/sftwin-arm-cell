"""Production gripper capture realization from the ARM Cell profile."""

import json
from pathlib import Path

from gripper_runtime.grasp_attachment import (
    CaptureReferenceConfig,
    EligibleObjectRegistry,
)
from gripper_runtime.grasp_policy import CaptureConfig


def load_production_mission_recipe(profile, profile_path):
    """Load the selected production recipe at the Isaac infrastructure edge."""
    orchestration = profile["orchestration"]
    target_id = orchestration["mission_target_id"]
    recipe_path = (
        Path(profile_path).parent
        / orchestration.get("recipe_directory", "recipes")
        / f"{target_id}.json"
    )
    recipe = json.loads(recipe_path.read_text(encoding="utf-8"))
    if recipe.get("target_id") != target_id:
        raise ValueError(
            f"production recipe target_id mismatch: expected {target_id!r}"
        )
    return recipe


def production_capture_configuration(profile, mission_recipe):
    """Derive capture geometry and contact width from production mission inputs."""
    raw_part = profile["vision"]["detector_profiles"]["RawPart"]
    motion = profile["motion"]
    capture = profile["gripper_capture"]
    dimensions = tuple(float(value) for value in raw_part["dimensions_m"])
    observation_to_object = tuple(
        float(value) for value in motion["observation_to_object_translation"]
    )
    translation = tuple(
        float(value) for value in motion["object_to_grasp_tcp_translation"]
    )
    insertion_axis = tuple(float(value) for value in motion["insertion_axis_tcp"])
    if len(translation) != 3:
        raise ValueError("object_to_grasp_tcp_translation must have three values")
    if len(observation_to_object) != 3:
        raise ValueError("observation_to_object_translation must have three values")
    if len(dimensions) != 3:
        raise ValueError("RawPart dimensions_m must have three values")
    reference = CaptureReferenceConfig(
        fixture_dimensions_m=dimensions,
        observation_to_grasp_tcp_m=observation_to_object[2] + translation[2],
        insertion_axis_tcp=insertion_axis,
    ).validate()
    config = CaptureConfig(
        capture_volume_dimensions_m=tuple(
            float(value) for value in capture["volume_dimensions_m"]
        ),
        position_tolerance_m=float(capture["position_tolerance_m"]),
        orientation_tolerance_deg=float(capture["orientation_tolerance_deg"]),
        expected_contact_width_mm=float(mission_recipe["grasp"]["width_mm"]),
        contact_width_tolerance_mm=float(capture["contact_width_tolerance_mm"]),
    ).validate()
    registry = EligibleObjectRegistry()
    registry.register("/World/SF_Twin_Cell/AMR/Mockup/RawPart")
    return config, reference, registry
