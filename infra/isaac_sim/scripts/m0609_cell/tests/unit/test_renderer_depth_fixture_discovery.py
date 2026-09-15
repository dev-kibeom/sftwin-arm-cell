from pathlib import Path
import json
import subprocess
import sys

import numpy as np
import pytest

from renderer_depth_fixture_discovery import (
    GRID_COLUMNS,
    GRID_ROWS,
    MAX_CANDIDATE_TRIPLES,
    characterize_depth,
    discover_fixture_layout,
    grid_pixels_for_target,
    pixel_to_optical_xyz,
    projected_footprint_clearance,
    triples_are_separated,
)


class CameraInfo:
    width = 640
    height = 480
    k = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]


def test_depth_characterization_counts_threshold_usable_pixels_and_coarse_regions():
    depth = np.array(
        [
            [np.nan, 0.0, 0.84, 0.85],
            [1.14, 1.15, 1.44, 1.45],
            [2.0, np.inf, 0.5, 1.2],
        ],
        dtype=np.float32,
    )
    info = type(
        "SmallInfo",
        (),
        {"width": 4, "height": 3, "k": [2.0, 0, 2.0, 0, 2.0, 1.5, 0, 0, 1]},
    )
    report = characterize_depth(depth, info)
    assert report["valid_depth_count"] == 9
    assert report["threshold_counts"]["0.85"]["count"] == 7
    assert report["threshold_counts"]["1.15"]["count"] == 5
    assert report["threshold_counts"]["1.45"]["count"] == 2
    assert "coarse_spatial_distribution_ge_1_45_m" in report


def test_projected_footprint_clearance_checks_border_finite_depth_and_margin():
    depth = np.full((CameraInfo.height, CameraInfo.width), 2.0, dtype=np.float32)
    assert projected_footprint_clearance(depth, CameraInfo, (320, 240), 0.8)["pass"]
    report = projected_footprint_clearance(depth, CameraInfo, (5, 5), 0.8)
    assert not report["pass"]
    assert "border_or_projected_footprint" in report["reasons"]


def test_finite_deterministic_grid_has_fixed_small_dimensions():
    depth = np.full((CameraInfo.height, CameraInfo.width), 2.0, dtype=np.float32)
    points, stats = grid_pixels_for_target(depth, CameraInfo, 0.8)
    assert len(points) <= GRID_COLUMNS * GRID_ROWS
    assert stats["grid_positions_evaluated"] == GRID_COLUMNS * GRID_ROWS
    assert points == grid_pixels_for_target(depth, CameraInfo, 0.8)[0]


def test_candidate_pixel_and_z_convert_to_optical_xyz_and_project_back():
    xyz = pixel_to_optical_xyz((200, 180), 1.1, CameraInfo)
    assert xyz == pytest.approx((-0.33, -0.165, 1.1))
    assert (CameraInfo.k[0] * xyz[0] / xyz[2] + CameraInfo.k[2]) == pytest.approx(200)
    assert (CameraInfo.k[4] * xyz[1] / xyz[2] + CameraInfo.k[5]) == pytest.approx(180)


def test_separation_filter_uses_existing_minimum_center_distance():
    assert triples_are_separated(((100, 100), (320, 100), (540, 100)))
    assert not triples_are_separated(((100, 100), (220, 100), (540, 100)))


def test_discovery_is_deterministic_and_candidate_count_is_hard_bounded():
    depth = np.full((CameraInfo.height, CameraInfo.width), 2.0, dtype=np.float32)
    first = discover_fixture_layout(depth, CameraInfo)
    second = discover_fixture_layout(depth, CameraInfo)
    assert first == second
    assert first["status"] == "FIXTURE SUITABILITY VERIFIED"
    assert first["candidate"] is not None
    json.dumps(first, allow_nan=False)
    search = first["search"]
    assert search["grid_dimensions"] == [GRID_COLUMNS, GRID_ROWS]
    assert search["candidate_triples_evaluated"] <= MAX_CANDIDATE_TRIPLES
    assert search["candidate_triples_evaluated"] <= search["candidate_triples_available"]
    assert first["candidate"]["targets"][0]["id"] == "z080"
    assert first["candidate"]["targets"][1]["id"] == "z110"
    assert first["candidate"]["targets"][2]["id"] == "z140"


def test_candidate_triple_evaluation_stops_at_explicit_hard_cap():
    info = type(
        "CompactInfo",
        (),
        {"width": 270, "height": 270, "k": [400.0, 0, 135.0, 0, 400.0, 135.0, 0, 0, 1]},
    )
    depth = np.full((info.height, info.width), 2.0, dtype=np.float32)
    report = discover_fixture_layout(depth, info)
    assert report["search"]["candidate_triples_available"] > MAX_CANDIDATE_TRIPLES
    assert report["search"]["candidate_triples_evaluated"] == MAX_CANDIDATE_TRIPLES
    assert report["status"] == "NOT VERIFIED"


def test_clearance_failure_filters_grid_pixels_without_threshold_weakening():
    depth = np.full((CameraInfo.height, CameraInfo.width), 2.0, dtype=np.float32)
    points, stats = grid_pixels_for_target(depth, CameraInfo, 1.4)
    assert points
    x, y = points[0].pixel_xy
    footprint = projected_footprint_clearance(depth, CameraInfo, (x, y), 1.4)
    left, top, right, bottom = footprint["footprint_bounds_xyxy"]
    depth[top : bottom + 1, left : right + 1] = 1.44
    reduced, reduced_stats = grid_pixels_for_target(depth, CameraInfo, 1.4)
    assert (x, y) not in [point.pixel_xy for point in reduced]
    assert (
        reduced_stats["pixel_rejection_counts"]["insufficient_clearance_in_footprint"]
        >= 1
    )
    assert stats["pixel_rejection_counts"]["accepted"] == len(points)


def test_no_z140_compatible_region_fails_without_expanding_search():
    depth = np.full((CameraInfo.height, CameraInfo.width), 1.2, dtype=np.float32)
    report = discover_fixture_layout(depth, CameraInfo)
    assert report["status"] == "NOT VERIFIED"
    assert report["candidate"] is None
    assert report["search"]["candidate_pixels_by_target"]["z140"] == 0
    assert report["search"]["candidate_triples_evaluated"] == 0
    assert report["search"]["maximum_candidate_triples"] == MAX_CANDIDATE_TRIPLES


def test_search_has_no_artifact_input_or_randomized_state():
    depth = np.full((CameraInfo.height, CameraInfo.width), 2.0, dtype=np.float32)
    report = discover_fixture_layout(depth, CameraInfo)
    assert "input_path" not in report
    assert "seed" not in report["search"]


def test_live_tool_entrypoint_help_runs_without_custom_pythonpath():
    script = (
        Path(__file__).resolve().parents[1]
        / "tooling"
        / "renderer_depth_fixture_discovery.py"
    )
    result = subprocess.run(
        [sys.executable, str(script), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0
    assert "--timeout-wall-s" in result.stdout
