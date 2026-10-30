import json
from pathlib import Path

import pytest
import yaml

from scene_builder.rawpart_profile import load_rawpart_profile, marker_black_cells


REPOSITORY = Path(__file__).resolve().parents[6]


def test_rawpart_profile_exposes_reviewed_production_geometry_and_support_relation():
    profile = load_rawpart_profile(REPOSITORY)

    assert profile["target_id"] == "RawPart"
    assert profile["material_id"] == "RawPart"
    assert profile["dimension_tolerance_m"] == [0.01, 0.01, 0.01]
    assert profile["support_contact_tolerance_m"] == 0.01
    assert profile["top_surface_geometry_tolerance_m"] == 0.01
    assert profile["fiducial"]["dictionary"] == "DICT_4X4_50"
    assert profile["fiducial"]["marker_id"] == 0
    assert profile["fiducial"]["size_m"] == 0.08
    assert profile["fiducial"]["pose_translation_scale"] == pytest.approx(1.0011)
    assert profile["fiducial"]["support_frame"] == "Tray"
    assert profile["fiducial"]["marker_in_support"]["position_m"]


def test_rawpart_scene_marker_is_the_configured_aruco_dictionary_code():
    profile = load_rawpart_profile(REPOSITORY)

    assert marker_black_cells(profile) == (
        (0, 0),
        (0, 1),
        (0, 2),
        (0, 3),
        (0, 4),
        (0, 5),
        (1, 0),
        (1, 2),
        (1, 5),
        (2, 0),
        (2, 1),
        (2, 3),
        (2, 5),
        (3, 0),
        (3, 1),
        (3, 2),
        (3, 5),
        (4, 0),
        (4, 1),
        (4, 2),
        (4, 4),
        (4, 5),
        (5, 0),
        (5, 1),
        (5, 2),
        (5, 3),
        (5, 4),
        (5, 5),
    )


def test_rawpart_profile_fails_closed_when_marker_relation_is_incomplete(tmp_path):
    support_dimensions = load_rawpart_profile(REPOSITORY)["support_region_dimensions_m"]
    config_path = (
        tmp_path
        / "ros2_ws/src/arm_cell_bringup/config/operational_profile.yaml"
    )
    config_path.parent.mkdir(parents=True)
    config_path.write_text(
        "material_definitions:\n"
        "  RawPart:\n"
        "    material_id: RawPart\n"
        "    shape: box\n"
        "    nominal_dimensions_m: [0.07, 0.07, 0.08]\n"
        "vision:\n"
        "  detector_profiles:\n"
        "    RawPart:\n"
        "      target_id: RawPart\n"
        "      material_id: RawPart\n"
        "      dimension_tolerance_m: [0.01, 0.01, 0.01]\n"
        "      workspace_roi_m:\n"
        "        x: [-0.12, 0.12]\n"
        "        y: [-0.14, 0.14]\n"
        "        z: [0.01, 0.12]\n"
        "      support_contact_tolerance_m: 0.01\n"
        "      top_surface_geometry_tolerance_m: 0.01\n"
        f"      support_region_dimensions_m: {support_dimensions}\n"
        "      fiducial:\n"
        "        dictionary: DICT_4X4_50\n"
        "        marker_id: 0\n"
        "        size_m: 0.08\n"
        "        support_frame: Tray\n"
        "  scene_stability:\n"
        "    roi_m:\n"
        "      x: [-0.30, 0.30]\n"
        "      y: [-0.20, 0.20]\n"
    )

    with pytest.raises(ValueError, match="marker_in_support"):
        load_rawpart_profile(tmp_path)


def _write_deployment(tmp_path, *, marker_y=None):
    profile_path = Path(
        "ros2_ws/src/arm_cell_bringup/config/operational_profile.yaml"
    )
    config = yaml.safe_load((REPOSITORY / profile_path).read_text())
    rawpart = config["vision"]["detector_profiles"]["RawPart"]
    rawpart["workspace_roi_m"]["x"] = [-0.3168, 0.3168]
    rawpart["workspace_roi_m"]["y"] = [-0.2208, 0.2208]
    if marker_y is not None:
        rawpart["fiducial"]["marker_in_support"]["position_m"][1] = marker_y
    destination = tmp_path / profile_path
    destination.parent.mkdir(parents=True)
    destination.write_text(yaml.safe_dump(config))


@pytest.mark.parametrize("marker_y", [-0.249, 0.0])
def test_tray_sized_detection_roi_loads_without_supply_config(tmp_path, marker_y):
    _write_deployment(tmp_path, marker_y=marker_y)
    assert load_rawpart_profile(tmp_path)["target_id"] == "RawPart"


def test_deployed_detection_roi_matches_canonical_tray_plate():
    scene_path = (
        REPOSITORY
        / "ros2_ws/src/arm_cell_moveit_config/config/m0609_static_scene.json"
    )
    objects = json.loads(scene_path.read_text())["objects"]
    tray = next(obj for obj in objects if obj["id"] == "amr_tray")
    dimensions = tray["geometry"]["dimensions_m"]
    profile = load_rawpart_profile(REPOSITORY)
    for axis, dimension in zip(("x", "y"), dimensions):
        assert profile["workspace_roi_m"][axis] == pytest.approx(
            [-dimension / 2, dimension / 2]
        )
