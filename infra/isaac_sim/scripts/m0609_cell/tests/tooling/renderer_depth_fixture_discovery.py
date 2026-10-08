"""Bounded deterministic discovery of renderer-fixture optical positions.

This is one-time tooling for a target-free live depth frame. It is not part of
normal acceptance and never authors or saves stage content.
"""

from dataclasses import dataclass
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

CELL_ROOT = Path(__file__).resolve().parents[2]
SUPPORT_ROOT = Path(__file__).resolve().parents[1] / "support"
for _module_root in (str(CELL_ROOT), str(SUPPORT_ROOT)):
    if _module_root not in sys.path:
        sys.path.insert(0, _module_root)

from camera_tooling.renderer_depth_target_spec import (
    CANONICAL_FRAME,
    TARGET_DEPTHS_M,
    TARGET_DIMENSIONS_M,
)


GRID_COLUMNS = 12
GRID_ROWS = 7
MAX_GRID_PIXELS_PER_TARGET = GRID_COLUMNS * GRID_ROWS
MAX_CANDIDATE_TRIPLES = 4096
MIN_BORDER_MARGIN_PX = 64
MINIMUM_INTERTARGET_SEPARATION_PX = 160.0
MINIMUM_DEPTH_CLEARANCE_M = 0.05
COARSE_DISTRIBUTION_COLUMNS = 8
COARSE_DISTRIBUTION_ROWS = 5
DEPTH_THRESHOLDS_M = tuple(
    depth + MINIMUM_DEPTH_CLEARANCE_M for depth in TARGET_DEPTHS_M
)


@dataclass(frozen=True)
class GridPixel:
    pixel_xy: tuple[int, int]
    baseline_depth_m: float
    footprint_min_depth_m: float
    footprint_bounds_xyxy: tuple[int, int, int, int]


def _validate_depth_and_camera(depth, camera_info):
    view = np.asarray(depth)
    if view.ndim != 2 or view.shape != (camera_info.height, camera_info.width):
        raise ValueError("depth raster dimensions must match CameraInfo")
    if len(camera_info.k) != 9 or not all(
        math.isfinite(float(value)) for value in camera_info.k
    ):
        raise ValueError("CameraInfo K must contain nine finite values")
    if float(camera_info.k[0]) <= 0 or float(camera_info.k[4]) <= 0:
        raise ValueError("CameraInfo focal lengths must be positive")
    return view


def _projected_half_footprint_px(camera_info, optical_z_m):
    width_m, height_m, _ = TARGET_DIMENSIONS_M
    return (
        int(math.ceil(float(camera_info.k[0]) * width_m / (2.0 * optical_z_m))),
        int(math.ceil(float(camera_info.k[4]) * height_m / (2.0 * optical_z_m))),
    )


def _usable_center_bounds(camera_info, optical_z_m):
    half_width, half_height = _projected_half_footprint_px(camera_info, optical_z_m)
    margin_x = MIN_BORDER_MARGIN_PX + half_width
    margin_y = MIN_BORDER_MARGIN_PX + half_height
    return (
        margin_x,
        margin_y,
        int(camera_info.width) - 1 - margin_x,
        int(camera_info.height) - 1 - margin_y,
    )


def _percentage(count, total):
    return 100.0 * count / total if total else 0.0


def characterize_depth(depth, camera_info):
    """Summarize valid/threshold depth and coarse deep-region distribution."""
    view = _validate_depth_and_camera(depth, camera_info)
    height, width = view.shape
    total = int(view.size)
    valid = np.isfinite(view) & (view > 0.0)
    threshold_counts = {}
    margin_usable = {}
    for threshold in DEPTH_THRESHOLDS_M:
        mask = valid & (view >= threshold)
        count = int(np.count_nonzero(mask))
        threshold_counts[f"{threshold:.2f}"] = {
            "count": count,
            "percent_of_image": _percentage(count, total),
        }
        z = threshold - MINIMUM_DEPTH_CLEARANCE_M
        x0, y0, x1, y1 = _usable_center_bounds(camera_info, z)
        center_mask = np.zeros(view.shape, dtype=bool)
        if x0 <= x1 and y0 <= y1:
            center_mask[y0 : y1 + 1, x0 : x1 + 1] = True
        usable_count = int(np.count_nonzero(mask & center_mask))
        margin_usable[f"z{int(round(z * 100)):03d}"] = {
            "count": usable_count,
            "percent_of_image": _percentage(usable_count, total),
            "center_bounds_xyxy": [x0, y0, x1, y1],
        }

    deepest_mask = valid & (view >= DEPTH_THRESHOLDS_M[-1])
    coarse_cells = []
    for row in range(COARSE_DISTRIBUTION_ROWS):
        y0 = height * row // COARSE_DISTRIBUTION_ROWS
        y1 = height * (row + 1) // COARSE_DISTRIBUTION_ROWS
        for column in range(COARSE_DISTRIBUTION_COLUMNS):
            x0 = width * column // COARSE_DISTRIBUTION_COLUMNS
            x1 = width * (column + 1) // COARSE_DISTRIBUTION_COLUMNS
            count = int(np.count_nonzero(deepest_mask[y0:y1, x0:x1]))
            coarse_cells.append(
                {
                    "grid_xy": [column, row],
                    "image_bounds_xyxy": [x0, y0, x1 - 1, y1 - 1],
                    "count_ge_1_45_m": count,
                }
            )
    return {
        "image_dimensions": {"width": width, "height": height},
        "valid_depth_definition": "finite positive depth; no additional far-range cutoff is configured",
        "valid_depth_count": int(np.count_nonzero(valid)),
        "valid_depth_percent": _percentage(int(np.count_nonzero(valid)), total),
        "image_pixel_count": total,
        "threshold_counts": threshold_counts,
        "usable_after_border_and_projected_footprint_margin": margin_usable,
        "coarse_spatial_distribution_ge_1_45_m": {
            "grid_dimensions": [COARSE_DISTRIBUTION_COLUMNS, COARSE_DISTRIBUTION_ROWS],
            "cells": coarse_cells,
        },
    }


def projected_footprint_clearance(depth, camera_info, pixel_xy, optical_z_m):
    """Require the complete projected target footprint to have clear background."""
    view = _validate_depth_and_camera(depth, camera_info)
    x, y = (int(pixel_xy[0]), int(pixel_xy[1]))
    half_width, half_height = _projected_half_footprint_px(camera_info, optical_z_m)
    left, top = x - half_width, y - half_height
    right, bottom = x + half_width, y + half_height
    reasons = []
    x0, y0, x1, y1 = _usable_center_bounds(camera_info, optical_z_m)
    if x < x0 or x > x1 or y < y0 or y > y1:
        reasons.append("border_or_projected_footprint")
    if left < 0 or top < 0 or right >= view.shape[1] or bottom >= view.shape[0]:
        return {
            "pass": False,
            "reasons": reasons or ["projected_footprint_out_of_frame"],
            "baseline_depth_m": None,
            "minimum_depth_m": None,
            "required_depth_m": optical_z_m + MINIMUM_DEPTH_CLEARANCE_M,
            "footprint_pixel_count": 0,
            "footprint_bounds_xyxy": [left, top, right, bottom],
        }

    footprint = view[top : bottom + 1, left : right + 1]
    finite_positive = np.isfinite(footprint) & (footprint > 0.0)
    valid_count = int(np.count_nonzero(finite_positive))
    finite_values = footprint[finite_positive]
    minimum_depth = float(np.min(finite_values)) if valid_count else None
    baseline_depth = float(view[y, x])
    required_depth = optical_z_m + MINIMUM_DEPTH_CLEARANCE_M
    if valid_count != footprint.size:
        reasons.append("nonfinite_or_nonpositive_depth_in_footprint")
    if valid_count != footprint.size or np.any(footprint < required_depth):
        reasons.append("insufficient_clearance_in_footprint")
    return {
        "pass": not reasons,
        "reasons": reasons,
        "baseline_depth_m": baseline_depth,
        "minimum_depth_m": minimum_depth,
        "required_depth_m": required_depth,
        "footprint_pixel_count": int(footprint.size),
        "footprint_bounds_xyxy": [left, top, right, bottom],
    }


def grid_pixels_for_target(depth, camera_info, optical_z_m):
    """Evaluate a fixed 12x7 center grid for one required target depth."""
    view = _validate_depth_and_camera(depth, camera_info)
    x0, y0, x1, y1 = _usable_center_bounds(camera_info, optical_z_m)
    rejected = {
        "border_or_projected_footprint": 0,
        "nonfinite_or_nonpositive_depth_in_footprint": 0,
        "insufficient_clearance_in_footprint": 0,
        "accepted": 0,
    }
    if x0 > x1 or y0 > y1:
        rejected["border_or_projected_footprint"] = GRID_COLUMNS * GRID_ROWS
        return [], {
            "grid_positions_evaluated": GRID_COLUMNS * GRID_ROWS,
            "pixel_rejection_counts": rejected,
        }

    x_values = np.rint(np.linspace(x0, x1, GRID_COLUMNS)).astype(int)
    y_values = np.rint(np.linspace(y0, y1, GRID_ROWS)).astype(int)
    positions = list(
        dict.fromkeys((int(x), int(y)) for y in y_values for x in x_values)
    )
    accepted = []
    for pixel in positions:
        check = projected_footprint_clearance(view, camera_info, pixel, optical_z_m)
        if check["pass"]:
            accepted.append(
                GridPixel(
                    pixel_xy=pixel,
                    baseline_depth_m=check["baseline_depth_m"],
                    footprint_min_depth_m=check["minimum_depth_m"],
                    footprint_bounds_xyxy=tuple(check["footprint_bounds_xyxy"]),
                )
            )
            rejected["accepted"] += 1
        else:
            for reason in set(check["reasons"]):
                rejected[reason] += 1
    return accepted, {
        "grid_positions_evaluated": len(positions),
        "pixel_rejection_counts": rejected,
    }


def pixel_to_optical_xyz(pixel_xy, optical_z_m, camera_info):
    """Convert an integer image pixel and fixed optical Z to optical-frame XYZ."""
    if optical_z_m <= 0:
        raise ValueError("optical Z must be positive")
    u, v = pixel_xy
    k = camera_info.k
    return (
        (float(u) - float(k[2])) * optical_z_m / float(k[0]),
        (float(v) - float(k[5])) * optical_z_m / float(k[4]),
        float(optical_z_m),
    )


def triples_are_separated(pixels, minimum_px=MINIMUM_INTERTARGET_SEPARATION_PX):
    if len(pixels) != 3:
        return False
    return all(
        math.dist(pixels[left], pixels[right]) >= minimum_px
        for left in range(3)
        for right in range(left + 1, 3)
    )


def _mixed_radix_indices(flat_index, counts):
    count_a, count_b, count_c = counts
    c_index = flat_index % count_c
    quotient = flat_index // count_c
    b_index = quotient % count_b
    a_index = quotient // count_b
    return a_index, b_index, c_index


def discover_fixture_layout(depth, camera_info, sensor_stamp_ns=None):
    """Return the first valid triple from a deterministic, hard-bounded search."""
    view = _validate_depth_and_camera(depth, camera_info)
    characterization = characterize_depth(view, camera_info)
    pixel_sets = []
    pixel_stats = {}
    for target_id, optical_z_m in zip(("z080", "z110", "z140"), TARGET_DEPTHS_M):
        points, stats = grid_pixels_for_target(view, camera_info, optical_z_m)
        pixel_sets.append(points)
        pixel_stats[target_id] = {
            "candidate_pixel_count": len(points),
            **stats,
        }

    counts = tuple(len(points) for points in pixel_sets)
    combinations_available = math.prod(counts)
    combinations_to_evaluate = min(combinations_available, MAX_CANDIDATE_TRIPLES)
    if combinations_available <= MAX_CANDIDATE_TRIPLES:
        flat_indices = range(combinations_to_evaluate)
    else:
        flat_indices = (
            (index * combinations_available) // combinations_to_evaluate
            for index in range(combinations_to_evaluate)
        )

    rejection_counts = {"insufficient_intertarget_separation": 0}
    accepted = None
    evaluated = 0
    for flat_index in flat_indices:
        selected_indices = _mixed_radix_indices(flat_index, counts)
        selected = tuple(
            pixel_sets[index][selected_indices[index]] for index in range(3)
        )
        pixels = tuple(item.pixel_xy for item in selected)
        evaluated += 1
        if not triples_are_separated(pixels):
            rejection_counts["insufficient_intertarget_separation"] += 1
            continue
        accepted = {
            "targets": [
                {
                    "id": target_id,
                    "frame_id": CANONICAL_FRAME,
                    "optical_xyz_m": list(pixel_to_optical_xyz(pixel, z, camera_info)),
                    "projected_pixel": list(pixel),
                    "baseline_depth_m": item.baseline_depth_m,
                    "footprint_min_depth_m": item.footprint_min_depth_m,
                    "required_baseline_depth_m": z + MINIMUM_DEPTH_CLEARANCE_M,
                    "projected_footprint_bounds_xyxy": list(item.footprint_bounds_xyxy),
                }
                for target_id, z, pixel, item in zip(
                    ("z080", "z110", "z140"),
                    TARGET_DEPTHS_M,
                    pixels,
                    selected,
                )
            ]
        }
        break

    if accepted is not None:
        rejection_counts["accepted"] = 1
    else:
        rejection_counts["accepted"] = 0

    grid_evaluations = sum(
        value["grid_positions_evaluated"] for value in pixel_stats.values()
    )
    return {
        "status": "FIXTURE SUITABILITY VERIFIED" if accepted else "NOT VERIFIED",
        "scope": "fixture placement suitability only; renderer correctness is not assessed",
        "sensor_stamp_ns": sensor_stamp_ns,
        "canonical_frame": CANONICAL_FRAME,
        "characterization": characterization,
        "search": {
            "grid_dimensions": [GRID_COLUMNS, GRID_ROWS],
            "maximum_grid_pixels_per_target": MAX_GRID_PIXELS_PER_TARGET,
            "grid_pixels_evaluated": grid_evaluations,
            "candidate_pixels_by_target": {
                target_id: stats["candidate_pixel_count"]
                for target_id, stats in pixel_stats.items()
            },
            "pixel_rejection_counts_by_target": {
                target_id: stats["pixel_rejection_counts"]
                for target_id, stats in pixel_stats.items()
            },
            "candidate_triples_available": combinations_available,
            "maximum_candidate_triples": MAX_CANDIDATE_TRIPLES,
            "candidate_triples_evaluated": evaluated,
            "triple_rejection_counts": rejection_counts,
            "limits": {
                "border_margin_px": MIN_BORDER_MARGIN_PX,
                "minimum_intertarget_separation_px": MINIMUM_INTERTARGET_SEPARATION_PX,
                "minimum_depth_clearance_m": MINIMUM_DEPTH_CLEARANCE_M,
            },
            "ordering": "raster-ordered grid product sampled uniformly by flat index when capped",
            "randomized": False,
        },
        "candidate": accepted,
    }


def _report_for_pair(depth_message, camera_info):
    from rgbd_assertions import (
        decode_depth_frame,
        stamp_nanoseconds,
        validate_camera_info,
    )

    if depth_message.encoding != "32FC1":
        raise ValueError("target-free baseline depth must use 32FC1")
    validate_camera_info(
        camera_info,
        depth_message.header.frame_id,
        (depth_message.width, depth_message.height),
    )
    if depth_message.header.frame_id != CANONICAL_FRAME:
        raise ValueError(f"depth frame must be {CANONICAL_FRAME}")
    return discover_fixture_layout(
        decode_depth_frame(depth_message),
        camera_info,
        sensor_stamp_ns=stamp_nanoseconds(depth_message.header.stamp),
    )


def _run_live(timeout_wall_s):
    try:
        import message_filters
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image
    except ModuleNotFoundError as error:
        raise SystemExit(
            "source /opt/ros/humble/setup.bash and ros2_ws/install/setup.bash before running this tool"
        ) from error

    class DiscoveryNode(Node):
        def __init__(self):
            super().__init__("renderer_depth_fixture_discovery")
            self.result = None
            self.depth_subscriber = message_filters.Subscriber(
                self, Image, "/camera/aligned_depth_to_color/image_raw"
            )
            self.info_subscriber = message_filters.Subscriber(
                self, CameraInfo, "/camera/color/camera_info"
            )
            self.synchronizer = message_filters.TimeSynchronizer(
                [self.depth_subscriber, self.info_subscriber], 10
            )
            self.synchronizer.registerCallback(self.on_pair)

        def on_pair(self, depth_message, camera_info):
            if self.result is None:
                try:
                    self.result = _report_for_pair(depth_message, camera_info)
                except (ValueError, TypeError) as error:
                    self.result = {
                        "status": "NOT VERIFIED",
                        "scope": "fixture placement suitability only; renderer correctness is not assessed",
                        "sensor_stamp_ns": None,
                        "error": str(error),
                        "characterization": None,
                        "search": None,
                        "candidate": None,
                    }

    rclpy.init()
    node = DiscoveryNode()
    deadline = time.monotonic() + timeout_wall_s
    try:
        while node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is None:
            return {
                "status": "NOT VERIFIED",
                "scope": "fixture placement suitability only; renderer correctness is not assessed",
                "reason": "wall timeout waiting for exact depth/CameraInfo pair",
                "characterization": None,
                "search": None,
                "candidate": None,
            }
        return node.result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-wall-s", type=float, default=20.0)
    parser.add_argument(
        "--diagnostic-dump",
        help="optional JSON output path; stdout is the default and no artifact is required",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout_wall_s) or args.timeout_wall_s <= 0:
        parser.error("--timeout-wall-s must be a positive finite value")
    report = _run_live(args.timeout_wall_s)
    rendered = json.dumps(report, indent=2, allow_nan=False)
    if args.diagnostic_dump:
        output = Path(args.diagnostic_dump).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n")
    print(rendered)
    return 0 if report["status"] == "FIXTURE SUITABILITY VERIFIED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
