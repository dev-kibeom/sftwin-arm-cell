import json
from pathlib import Path

import pytest
import yaml
from pnp_validation.profile import (
    capture_config_from_profile,
    capture_reference_config_from_profile,
    load_validation_profile,
    validate_validation_profile,
)


@pytest.mark.parametrize(
    "pick",
    [
        None,
        [],
        {},
        {"approach_distance_m": None},
        {"approach_distance_m": "0.05"},
        {"approach_distance_m": True},
        {"approach_distance_m": []},
        {"approach_distance_m": 0.0},
        {"approach_distance_m": -0.05},
        {"approach_distance_m": 0.1},
        {"approach_distance_m": float("nan")},
        {"approach_distance_m": float("inf")},
    ],
)
def test_validation_profile_loader_rejects_missing_or_invalid_recipe_approach(
    tmp_path, pick
):
    config_directory = (
        Path(__file__).resolve().parents[6]
        / "ros2_ws/src/arm_cell_bringup/config"
    )
    profile_data = yaml.safe_load(
        (config_directory / "pnp_validation_profile.yaml").read_text()
    )
    recipe_directory = profile_data["orchestration"]["recipe_directory"]
    target_id = profile_data["orchestration"]["mission_target_id"]
    recipe_data = json.loads(
        (config_directory / recipe_directory / f"{target_id}.json").read_text()
    )
    if pick is None:
        del recipe_data["pick"]
    else:
        recipe_data["pick"] = pick
    profile_path = tmp_path / "validation.yaml"
    profile_path.write_text(yaml.safe_dump(profile_data))
    (tmp_path / recipe_directory).mkdir()
    (tmp_path / recipe_directory / f"{target_id}.json").write_text(
        json.dumps(recipe_data)
    )

    with pytest.raises(ValueError, match=r"recipe pick\.approach_distance_m"):
        load_validation_profile(profile_path)


def test_validation_profile_declares_world_geometry_and_place_tolerance():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())

    assert profile_data["fixture"]["geometry_frame"] == "world"
    assert profile_data["place"]["position_tolerance_m"] == pytest.approx(0.05)
    assert validate_validation_profile(profile_data) is profile_data
    assert profile_data["fixture"]["support_surface_id"] == (
        "/World/SF_Twin_Cell/AMR/Mockup/Tray"
    )
    assert profile_data["fixture"]["support_surface_frame"] == "amr_top"


def test_validation_profile_rejects_missing_geometry_frame():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())
    del profile_data["fixture"]["geometry_frame"]

    with pytest.raises(ValueError, match="geometry_frame"):
        validate_validation_profile(profile_data)


def test_validation_profile_rejects_missing_or_invalid_place_tolerance():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())
    del profile_data["place"]["position_tolerance_m"]

    with pytest.raises(ValueError, match="position_tolerance_m"):
        validate_validation_profile(profile_data)

    profile_data = yaml.safe_load(path.read_text())
    profile_data["place"]["position_tolerance_m"] = 0.0
    with pytest.raises(ValueError, match="position_tolerance_m"):
        validate_validation_profile(profile_data)

    profile_data = yaml.safe_load(path.read_text())
    profile_data["fixture"]["geometry_frame"] = "map"
    with pytest.raises(ValueError, match="geometry_frame"):
        validate_validation_profile(profile_data)


def test_validation_profile_rejects_grasp_offset_drift_between_motion_and_recipe():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())
    profile_data["motion"]["object_to_grasp_tcp_translation"][2] = 0.0

    with pytest.raises(ValueError, match="offset"):
        validate_validation_profile(profile_data)


def test_validation_profile_rejects_nonzero_grasp_translation_xy():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())
    profile_data["motion"]["object_to_grasp_tcp_translation"][0] = 0.001

    with pytest.raises(ValueError, match="x must be zero"):
        validate_validation_profile(profile_data)


def test_capture_profile_maps_to_task3_capture_config_in_degrees():
    config = capture_config_from_profile(
        {
            "capture": {
                "volume_dimensions_m": [0.12, 0.12, 0.12],
                "position_tolerance_m": 0.015,
                "orientation_tolerance_deg": 10.0,
                "contact_width_tolerance_mm": 2.0,
            }
        }
    )

    assert config.capture_volume_dimensions_m == (0.12, 0.12, 0.12)
    assert config.validate() is config


def test_capture_reference_profile_maps_fixture_geometry_and_motion_semantics():
    config = capture_reference_config_from_profile(
        {
            "fixture": {"dimensions_m": [0.07, 0.07, 0.08]},
            "motion": {
                "observation_reference": "fixture_profile_reference",
                "observation_to_object_translation": [0.0, 0.0, -0.04],
                "object_to_grasp_tcp_translation": [0.0, 0.0, 0.0375],
                "insertion_axis_tcp": [0.0, 0.0, 1.0],
            },
        }
    )

    assert config.fixture_dimensions_m == (0.07, 0.07, 0.08)
    assert config.observation_to_grasp_tcp_m == pytest.approx(-0.0025)
    assert config.insertion_axis_tcp == (0.0, 0.0, 1.0)
