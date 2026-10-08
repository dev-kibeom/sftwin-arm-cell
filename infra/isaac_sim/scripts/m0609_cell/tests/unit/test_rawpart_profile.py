from pathlib import Path

import pytest

from scene_builder.rawpart_profile import load_rawpart_profile, marker_black_cells


REPOSITORY = Path(__file__).resolve().parents[6]


def test_rawpart_profile_exposes_reviewed_production_geometry_and_support_relation():
    profile = load_rawpart_profile(REPOSITORY)

    assert profile["target_id"] == "RawPart"
    assert profile["shape"] == "box"
    assert profile["dimensions_m"] == [0.07, 0.07, 0.08]
    assert profile["dimension_tolerance_m"] == [0.01, 0.01, 0.01]
    assert profile["workspace_roi_m"] == {
        "x": [-0.12, 0.12],
        "y": [-0.14, 0.14],
        "z": [0.01, 0.12],
    }
    assert profile["support_contact_tolerance_m"] == 0.01
    assert profile["top_surface_geometry_tolerance_m"] == 0.01
    assert profile["fiducial"]["dictionary"] == "DICT_4X4_50"
    assert profile["fiducial"]["marker_id"] == 0
    assert profile["fiducial"]["size_m"] == 0.04
    assert profile["fiducial"]["support_frame"] == "RawSlot"


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
    config_path = (
        tmp_path
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/operational_profile.yaml"
    )
    config_path.parent.mkdir(parents=True)
    config_path.write_text(
        "vision:\n"
        "  detector_profiles:\n"
        "    RawPart:\n"
        "      target_id: RawPart\n"
        "      shape: box\n"
        "      dimensions_m: [0.07, 0.07, 0.08]\n"
        "      dimension_tolerance_m: [0.01, 0.01, 0.01]\n"
        "      workspace_roi_m:\n"
        "        x: [-0.12, 0.12]\n"
        "        y: [-0.14, 0.14]\n"
        "        z: [0.01, 0.12]\n"
        "      support_contact_tolerance_m: 0.01\n"
        "      top_surface_geometry_tolerance_m: 0.01\n"
        "      support_region_dimensions_m: [0.24, 0.28]\n"
        "      fiducial:\n"
        "        dictionary: DICT_4X4_50\n"
        "        marker_id: 0\n"
        "        size_m: 0.04\n"
        "        support_frame: RawSlot\n"
    )

    with pytest.raises(ValueError, match="marker_in_support"):
        load_rawpart_profile(tmp_path)
