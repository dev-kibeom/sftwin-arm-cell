import math

from live_geometry_acceptance import (
    collision_query_entry,
    geometry_intermediates,
    point_diagnostics,
    usd_camera_direct_optical,
)


SNAPSHOT = {
    "world_to_usd_camera": {
        "translation_m": [1.0, 2.0, 3.0],
        "quaternion_xyzw": [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
    },
    "world_to_camera_link": {
        "translation_m": [1.0, 2.0, 3.0],
        "quaternion_xyzw": [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
    },
    "camera_link_to_optical": {
        "translation_m": [0.0, 0.0, 0.0],
        "quaternion_xyzw": [0.5, -0.5, 0.5, -0.5],
    },
    "usd_camera_to_ros_optical_point_coordinate_basis": (
        (1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, -1.0),
    ),
}


def test_nonidentity_camera_pose_uses_inverse_pose_for_world_point_conversion():
    link, optical = geometry_intermediates([1.0, 3.0, 3.0], SNAPSHOT)
    assert link == pytest.approx((1.0, 0.0, 0.0))
    assert optical[2] == pytest.approx(1.0)


def test_nonidentity_camera_pose_marks_point_behind_camera():
    diagnostic = point_diagnostics([1.0, 1.0, 3.0], SNAPSHOT)
    assert diagnostic["optical_z"] == pytest.approx(-1.0)


def test_direct_usd_camera_path_has_positive_forward_depth_for_front_point():
    usd, optical = usd_camera_direct_optical([1.0, 2.0, 2.0], SNAPSHOT)
    assert usd == pytest.approx((0.0, 0.0, -1.0))
    assert optical == pytest.approx((0.0, 0.0, 1.0))


def test_collision_query_entry_matches_reference_prim_and_point_without_claiming_renderer_truth():
    snapshot = {
        "collision_scene_query": {
            "entries": [
                {
                    "reference_prim_path": "/World/RawPart",
                    "reference_point": "top_center",
                    "hit_optical_z_m": 1.02,
                    "limitation": "Collision hit is not accepted as Replicator renderer-visible surface ground truth.",
                }
            ]
        }
    }
    entry = collision_query_entry(snapshot, "/World/RawPart", "top_center")
    assert entry["hit_optical_z_m"] == 1.02
    assert collision_query_entry(snapshot, "/World/RawPart", "top_off_axis_a") is None


import pytest


def test_geometry_diagnostic_default_is_repo_local_and_override_creates_parent(
    tmp_path,
):
    from live_geometry_acceptance import diagnostic_output_directory

    default = diagnostic_output_directory()
    assert str(default).endswith(".local_artifacts/m0609_cell/live_geometry_acceptance")
    override = diagnostic_output_directory(tmp_path / "nested" / "diagnostics")
    assert override == tmp_path / "nested" / "diagnostics"
    assert override.is_dir()


def test_geometry_diagnostic_artifacts_use_stable_filenames(tmp_path):
    from types import SimpleNamespace

    import numpy as np
    from live_geometry_acceptance import save_diagnostic_artifacts

    rgb = SimpleNamespace(width=2, height=1, step=6, data=bytes([1, 2, 3, 4, 5, 6]))
    artifacts = save_diagnostic_artifacts(
        tmp_path,
        123,
        np.array([[1.0, 2.0]], dtype=np.float32),
        rgb,
        [{"pixel": (0, 0)}],
    )
    assert artifacts == {
        "depth_npy": str(tmp_path / "depth_latest.npy"),
        "rgb_overlay_ppm": str(tmp_path / "rgb_projected_reference_overlay_latest.ppm"),
    }
    assert (tmp_path / "depth_latest.npy").is_file()
    assert (tmp_path / "rgb_projected_reference_overlay_latest.ppm").is_file()
