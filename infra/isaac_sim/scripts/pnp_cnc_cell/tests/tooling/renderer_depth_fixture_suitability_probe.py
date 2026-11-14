"""Bounded, non-mutating live suitability probe for renderer target layouts.

This probe checks visibility/clearance opportunities in the target-free live
RGB-D scene. It does not author USD, use PhysX, or verify renderer semantics.
"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

CELL_ROOT = Path(__file__).resolve().parents[2]
SUPPORT_ROOT = Path(__file__).resolve().parents[1] / "support"
for _path in (str(CELL_ROOT), str(SUPPORT_ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from camera_tooling.renderer_depth_target_spec import (  # noqa: E402
    CANDIDATE_A,
    CANONICAL_FRAME,
    CandidateSet,
    TARGET_DEPTHS_M,
    fallback_candidates,
)
from rgbd_assertions import (  # noqa: E402
    decode_depth_frame,
    stamp_nanoseconds,
    validate_camera_info,
)


DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"


@dataclass(frozen=True)
class SuitabilityCriteria:
    border_margin_px: int = 64
    minimum_separation_px: float = 160.0
    minimum_depth_clearance_m: float = 0.05


def project_target(target, camera_info):
    x, y, z = target.front_optical_xyz_m
    if z <= 0:
        raise ValueError(f"target {target.target_id} must be in front of the camera")
    k = camera_info.k
    return float(k[0] * x / z + k[2]), float(k[4] * y / z + k[5])


def _evaluate_target(target, decoded_depth, camera_info, criteria):
    image_height, image_width = decoded_depth.shape
    u, v = project_target(target, camera_info)
    pixel = (int(round(u)), int(round(v)))
    target_width_m, target_height_m, _ = target.dimensions_m
    half_width_px = (
        camera_info.k[0] * target_width_m / (2.0 * target.front_optical_xyz_m[2])
    )
    half_height_px = (
        camera_info.k[4] * target_height_m / (2.0 * target.front_optical_xyz_m[2])
    )
    required_border_x = criteria.border_margin_px + half_width_px
    required_border_y = criteria.border_margin_px + half_height_px

    reasons = []
    inside_image = 0 <= pixel[0] < image_width and 0 <= pixel[1] < image_height
    if not inside_image:
        reasons.append("projected pixel outside image")
    border_pass = inside_image and (
        pixel[0] >= required_border_x
        and pixel[0] <= image_width - 1 - required_border_x
        and pixel[1] >= required_border_y
        and pixel[1] <= image_height - 1 - required_border_y
    )
    if inside_image and not border_pass:
        reasons.append("projected target lacks required border margin")

    baseline_depth = float(decoded_depth[pixel[1], pixel[0]]) if inside_image else None
    required_depth = target.front_optical_xyz_m[2] + criteria.minimum_depth_clearance_m
    depth_clearance_pass = (
        baseline_depth is not None
        and math.isfinite(baseline_depth)
        and baseline_depth >= required_depth
    )
    if inside_image and not depth_clearance_pass:
        reasons.append("target-free baseline lacks required depth clearance")

    return {
        "id": target.target_id,
        "optical_xyz_m": list(target.front_optical_xyz_m),
        "projected_pixel_float": [u, v],
        "projected_pixel": list(pixel),
        "baseline_depth_m_at_pixel": baseline_depth,
        "required_depth_clearance_m": criteria.minimum_depth_clearance_m,
        "required_baseline_depth_m": required_depth,
        "border_margin_px": criteria.border_margin_px,
        "projected_half_target_size_px": [half_width_px, half_height_px],
        "border_pass": border_pass,
        "depth_clearance_pass": depth_clearance_pass,
        "reasons": reasons,
        "status": "PASS" if not reasons else "FAIL",
    }


def evaluate_candidate(candidate, depth, camera_info, criteria=SuitabilityCriteria()):
    """Evaluate one immutable candidate against a depth raster and live intrinsics."""
    decoded = np.asarray(depth)
    if decoded.ndim != 2 or decoded.shape != (camera_info.height, camera_info.width):
        raise ValueError("depth raster dimensions must match CameraInfo")
    if len(candidate.targets) != 3:
        raise ValueError("renderer fixture candidate must define exactly three targets")
    if any(target.frame_id != CANONICAL_FRAME for target in candidate.targets):
        raise ValueError(f"candidate targets must use {CANONICAL_FRAME}")
    if len(camera_info.k) != 9:
        raise ValueError("CameraInfo K must contain nine values")
    if not all(math.isfinite(float(value)) for value in camera_info.k):
        raise ValueError("CameraInfo K must contain finite values")
    if float(camera_info.k[0]) <= 0 or float(camera_info.k[4]) <= 0:
        raise ValueError("CameraInfo focal lengths must be positive")
    if (
        criteria.border_margin_px < 0
        or criteria.minimum_separation_px <= 0
        or criteria.minimum_depth_clearance_m < 0
    ):
        raise ValueError(
            "suitability criteria must be non-negative and separation positive"
        )
    expected_z = tuple(target.front_optical_xyz_m[2] for target in candidate.targets)
    if expected_z != TARGET_DEPTHS_M:
        raise ValueError("candidate optical depths must be 0.80, 1.10, and 1.40 metres")

    entries = [
        _evaluate_target(target, decoded, camera_info, criteria)
        for target in candidate.targets
    ]
    separations = []
    separation_pass = True
    for left_index in range(len(entries)):
        for right_index in range(left_index + 1, len(entries)):
            left = entries[left_index]["projected_pixel_float"]
            right = entries[right_index]["projected_pixel_float"]
            distance = math.dist(left, right)
            passed = distance >= criteria.minimum_separation_px
            separation_pass = separation_pass and passed
            separations.append(
                {
                    "target_ids": [
                        entries[left_index]["id"],
                        entries[right_index]["id"],
                    ],
                    "distance_px": distance,
                    "required_px": criteria.minimum_separation_px,
                    "pass": passed,
                }
            )
    if not separation_pass:
        for target in entries:
            target["reasons"].append("inter-target pixel separation is insufficient")
            target["status"] = "FAIL"

    passed = separation_pass and all(target["status"] == "PASS" for target in entries)
    return {
        "candidate_id": candidate.candidate_id,
        "source": candidate.source,
        "status": "PASS" if passed else "FAIL",
        "criteria": {
            "border_margin_px": criteria.border_margin_px,
            "minimum_separation_px": criteria.minimum_separation_px,
            "minimum_depth_clearance_m": criteria.minimum_depth_clearance_m,
            "depth_sample": "target-free renderer depth at rounded projected center pixel",
        },
        "separation_pass": separation_pass,
        "separations": separations,
        "targets": entries,
    }


def evaluate_candidates(depth, camera_info, criteria=SuitabilityCriteria()):
    """Evaluate Candidate A first and stop immediately on its first PASS."""
    evaluated = [evaluate_candidate(CANDIDATE_A, depth, camera_info, criteria)]
    if evaluated[0]["status"] == "PASS":
        return {
            "status": "PASS",
            "scope": "fixture suitability only; renderer correctness is not assessed",
            "sample_stamp_ns": None,
            "evaluated_candidates": evaluated,
            "fallback_candidates": [
                {"candidate_id": candidate_id, "status": "NOT EVALUATED"}
                for candidate_id in ("B", "C")
            ],
            "final_recommendation": "A",
        }

    candidates = fallback_candidates(camera_info)
    recommendation = None
    for candidate in candidates:
        result = evaluate_candidate(candidate, depth, camera_info, criteria)
        evaluated.append(result)
        if result["status"] == "PASS":
            recommendation = candidate.candidate_id
            break
    evaluated_ids = {entry["candidate_id"] for entry in evaluated}
    return {
        "status": "PASS" if recommendation else "FAIL",
        "scope": "fixture suitability only; renderer correctness is not assessed",
        "sample_stamp_ns": None,
        "evaluated_candidates": evaluated,
        "fallback_candidates": [
            {"candidate_id": candidate.candidate_id, "status": "NOT EVALUATED"}
            for candidate in candidates
            if candidate.candidate_id not in evaluated_ids
        ],
        "final_recommendation": recommendation,
    }


def report_for_sample(depth_message, camera_info):
    validate_camera_info(
        camera_info,
        depth_message.header.frame_id,
        (depth_message.width, depth_message.height),
    )
    report = evaluate_candidates(decode_depth_frame(depth_message), camera_info)
    report["status"] = (
        "FIXTURE SUITABILITY VERIFIED"
        if report["final_recommendation"] is not None
        else "NOT VERIFIED"
    )
    report["sample_stamp_ns"] = stamp_nanoseconds(depth_message.header.stamp)
    report["frame_id"] = depth_message.header.frame_id
    return report


def write_diagnostic_dump(report, output_path):
    """Write optional evidence without changing the in-memory decision."""
    path = Path(output_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(serialize_report(report) + "\n")
    return path


def _json_compatible(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return [_json_compatible(item) for item in value.tolist()]
    if isinstance(value, dict):
        return {key: _json_compatible(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_compatible(item) for item in value]
    return value


def serialize_report(report):
    """Serialize JSON-compatible report values, including NumPy scalars."""
    return json.dumps(_json_compatible(report), indent=2)


def _run_live(timeout_wall_s):
    try:
        import message_filters
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image
    except ModuleNotFoundError as error:
        raise SystemExit(
            "source /opt/ros/humble/setup.bash before running the live probe"
        ) from error

    class ProbeNode(Node):
        def __init__(self):
            super().__init__("renderer_depth_fixture_suitability_probe")
            self.result = None
            self.depth_subscriber = message_filters.Subscriber(self, Image, DEPTH_TOPIC)
            self.info_subscriber = message_filters.Subscriber(
                self, CameraInfo, CAMERA_INFO_TOPIC
            )
            self.synchronizer = message_filters.TimeSynchronizer(
                [self.depth_subscriber, self.info_subscriber], 10
            )
            self.synchronizer.registerCallback(self.on_pair)

        def on_pair(self, depth_message, camera_info):
            if self.result is None:
                try:
                    self.result = report_for_sample(depth_message, camera_info)
                except (ValueError, TypeError) as error:
                    self.result = {
                        "status": "NOT VERIFIED",
                        "reason": f"invalid live depth/CameraInfo sample: {error}",
                        "sample_stamp_ns": stamp_nanoseconds(
                            depth_message.header.stamp
                        ),
                        "evaluated_candidates": [],
                        "fallback_candidates": [],
                        "final_recommendation": None,
                    }

    rclpy.init()
    node = ProbeNode()
    deadline = time.monotonic() + timeout_wall_s
    try:
        while node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.result is None:
            return {
                "status": "NOT VERIFIED",
                "reason": "wall timeout waiting for exact target-free depth/CameraInfo pair",
                "evaluated_candidates": [],
                "fallback_candidates": [],
                "final_recommendation": None,
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
        help="optional JSON evidence output path; no artifact is written by default",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout_wall_s) or args.timeout_wall_s <= 0:
        parser.error("--timeout-wall-s must be a positive finite value")
    report = _run_live(args.timeout_wall_s)
    if args.diagnostic_dump:
        report["diagnostic_dump_path"] = str(
            write_diagnostic_dump(report, args.diagnostic_dump)
        )
    print(serialize_report(report))
    return 0 if report["status"] == "FIXTURE SUITABILITY VERIFIED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
