"""One-shot live RGB-D geometry acceptance; no ground truth comes from CameraInfo/depth."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

SUPPORT = Path(__file__).resolve().parents[1] / "support"
CELL_MODULE_ROOT = Path(__file__).resolve().parents[2]
for path in (CELL_MODULE_ROOT, SUPPORT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener
except ModuleNotFoundError as error:
    if error.name in {"rclpy", "sensor_msgs", "tf2_ros"}:
        raise SystemExit(
            "ROS 2 Python packages are unavailable. Run `source /opt/ros/humble/setup.bash` "
            "and `source ros2_ws/install/setup.bash` before this acceptance command."
        ) from error
    raise

from rgbd_assertions import (
    coarse_depth_grid,
    decode_depth_frame,
    depth_coordinate_hypotheses,
    depth_frame_distribution,
    depth_patch_summary,
    depth_value_at,
    stamp_nanoseconds,
    validate_camera_info,
    validate_image_payload,
)
from depth_semantics import classify_depth_samples
from shared.local_artifacts import default_artifact_directory


RGB = "/camera/color/image_raw"
DEPTH = "/camera/aligned_depth_to_color/image_raw"
INFO = "/camera/color/camera_info"


def q_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_multiply(a, b):
    return (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )


def q_rotate(q, point):
    result = q_multiply(q_multiply(q, (*point, 0.0)), q_conjugate(q))
    return result[:3]


def parent_to_child(point, transform):
    shifted = [point[i] - transform["translation_m"][i] for i in range(3)]
    return q_rotate(q_conjugate(transform["quaternion_xyzw"]), shifted)


def world_to_optical(point, snapshot):
    return parent_to_child(
        parent_to_child(point, snapshot["world_to_camera_link"]),
        snapshot["camera_link_to_optical"],
    )


def usd_camera_direct_optical(point, snapshot):
    usd = parent_to_child(point, snapshot["world_to_usd_camera"])
    basis = snapshot["usd_camera_to_ros_optical_point_coordinate_basis"]
    return usd, matrix_vector(basis, usd)


def geometry_intermediates(point, snapshot):
    link = parent_to_child(point, snapshot["world_to_camera_link"])
    optical = parent_to_child(link, snapshot["camera_link_to_optical"])
    return link, optical


def rotation_matrix_from_quaternion(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def matrix_vector(matrix, vector):
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def matrix_multiply(left, right):
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def point_diagnostics(point, snapshot):
    origin = snapshot["world_to_camera_link"]["translation_m"]
    link, optical = geometry_intermediates(point, snapshot)
    z = optical[2]
    return {
        "world_xyz": point,
        "camera_origin_world": origin,
        "world_delta": [point[i] - origin[i] for i in range(3)],
        "camera_link_xyz": link,
        "optical_xyz": optical,
        "optical_z": z,
        "x_over_z": optical[0] / z if z else None,
        "y_over_z": optical[1] / z if z else None,
    }


def project(point, info):
    return (
        info.k[0] * point[0] / point[2] + info.k[2],
        info.k[4] * point[1] / point[2] + info.k[5],
    )


def quaternion_angle(a, b):
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def collision_query_entry(snapshot, reference_prim_path, point_name):
    report = snapshot.get("collision_scene_query", {})
    for entry in report.get("entries", []):
        if (
            entry.get("reference_prim_path") == reference_prim_path
            and entry.get("reference_point") == point_name
        ):
            return entry
    return None


def average_ranks(values):
    ranks = [0.0] * len(values)
    for rank, (_, original_index) in enumerate(
        sorted((value, index) for index, value in enumerate(values)), start=1
    ):
        ranks[original_index] = rank
    for value in set(values):
        matching = [
            index for index, candidate in enumerate(values) if candidate == value
        ]
        mean_rank = sum(ranks[index] for index in matching) / len(matching)
        for index in matching:
            ranks[index] = mean_rank
    return ranks


def ordering_diagnostic(entries):
    ordered = sorted(entries, key=lambda entry: entry["expected_optical_z_m"])
    observed = [entry["observed_depth"] for entry in ordered]
    expected = [entry["expected_optical_z_m"] for entry in ordered]
    if len(observed) < 2 or len(set(observed)) < 2 or len(set(expected)) < 2:
        correlation = None
    else:
        correlation = float(
            np.corrcoef(average_ranks(expected), average_ranks(observed))[0, 1]
        )
    return {
        "sample_count": len(ordered),
        "ordered_points": [entry["point"] for entry in ordered],
        "expected_optical_z_m": expected,
        "observed_depth": observed,
        "observed_non_decreasing_with_expected_z": all(
            left <= right for left, right in zip(observed, observed[1:])
        ),
        "spearman_rank_correlation": correlation,
    }


def diagnostic_output_directory(override=None, script_path=None, environ=None):
    """Resolve an explicit diagnostic directory or the stable local default."""
    directory = (
        Path(override).expanduser()
        if override
        else default_artifact_directory(
            "live_geometry_acceptance",
            script_path if script_path is not None else __file__,
            environ,
        )
    )
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def save_diagnostic_artifacts(output_dir, stamp, depth_view, rgb, measurements):
    """Write an unchanged depth array and an RGB overlay for offline inspection."""
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    depth_path = directory / "depth_latest.npy"
    np.save(depth_path, depth_view)
    rgb_view = np.ndarray(
        shape=(rgb.height, rgb.width, 3),
        dtype=np.uint8,
        buffer=memoryview(rgb.data),
        strides=(rgb.step, 3, 1),
    )
    overlay = np.array(rgb_view, copy=True)
    for measurement in measurements:
        pixel = measurement.get("pixel")
        if not pixel:
            continue
        column, row = pixel
        overlay[max(0, row - 3) : min(rgb.height, row + 4), column] = [255, 0, 0]
        overlay[row, max(0, column - 3) : min(rgb.width, column + 4)] = [255, 0, 0]
    overlay_path = directory / "rgb_projected_reference_overlay_latest.ppm"
    with overlay_path.open("wb") as stream:
        stream.write(f"P6\n{rgb.width} {rgb.height}\n255\n".encode())
        stream.write(overlay.tobytes())
    return {"depth_npy": str(depth_path), "rgb_overlay_ppm": str(overlay_path)}


class Acceptance(Node):
    def __init__(
        self,
        snapshot,
        depth_tolerance,
        normalized_depth_tolerance,
        tf_translation_tolerance,
        tf_rotation_tolerance,
        diagnostic_output_dir=None,
    ):
        super().__init__("rgbd_geometry_acceptance")
        self.snapshot, self.depth_tolerance = snapshot, depth_tolerance
        self.normalized_depth_tolerance = normalized_depth_tolerance
        self.diagnostic_output_dir = diagnostic_output_dir
        self.tf_translation_tolerance, self.tf_rotation_tolerance = (
            tf_translation_tolerance,
            tf_rotation_tolerance,
        )
        self.info = {}
        self.rgb = {}
        self.depth = {}
        self.result = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(CameraInfo, INFO, self.on_info, 10)
        self.create_subscription(Image, RGB, self.on_rgb, 10)
        self.create_subscription(Image, DEPTH, self.on_depth, 10)

    def on_info(self, message):
        self.info[stamp_nanoseconds(message.header.stamp)] = message

    def on_rgb(self, message):
        self.rgb[stamp_nanoseconds(message.header.stamp)] = message
        self.try_pair(message)

    def on_depth(self, message):
        self.depth[stamp_nanoseconds(message.header.stamp)] = message
        self.try_pair(message)

    def tf_check(self, parent, child, expected):
        transform = self.buffer.lookup_transform(
            parent, child, rclpy.time.Time()
        ).transform
        actual_t = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ]
        actual_q = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        translation_error = math.dist(actual_t, expected["translation_m"])
        rotation_error = quaternion_angle(actual_q, expected["quaternion_xyzw"])
        return {
            "translation_error_m": translation_error,
            "rotation_error_rad": rotation_error,
            "status": (
                "VERIFIED"
                if translation_error <= self.tf_translation_tolerance
                and rotation_error <= self.tf_rotation_tolerance
                else "NOT VERIFIED"
            ),
        }

    def try_pair(self, message):
        stamp = stamp_nanoseconds(message.header.stamp)
        if (
            self.result is not None
            or stamp not in self.rgb
            or stamp not in self.depth
            or stamp not in self.info
        ):
            return
        rgb, depth, info = self.rgb[stamp], self.depth[stamp], self.info[stamp]
        validate_image_payload(rgb)
        validate_camera_info(info, rgb.header.frame_id, (rgb.width, rgb.height))
        decoded_depth = decode_depth_frame(depth)
        try:
            tf = {
                "world_to_camera_link": self.tf_check(
                    "world", "camera_link", self.snapshot["world_to_camera_link"]
                ),
                "camera_link_to_optical": self.tf_check(
                    "camera_link",
                    "camera_color_optical_frame",
                    self.snapshot["camera_link_to_optical"],
                ),
            }
        except Exception as error:
            self.result = {
                "status": "NOT VERIFIED",
                "reason": f"TF lookup failed: {error}",
            }
            return
        pose_rotation = rotation_matrix_from_quaternion(
            self.snapshot["camera_link_to_optical"]["quaternion_xyzw"]
        )
        used_rotation = tuple(zip(*pose_rotation))
        world_link_rotation = rotation_matrix_from_quaternion(
            self.snapshot["world_to_camera_link"]["quaternion_xyzw"]
        )
        direct_rotation = rotation_matrix_from_quaternion(
            self.snapshot["world_to_optical"]["quaternion_xyzw"]
        )
        reconstructed_rotation = matrix_multiply(world_link_rotation, pose_rotation)
        reconstruction_rotation_error = max(
            abs(reconstructed_rotation[row][column] - direct_rotation[row][column])
            for row in range(3)
            for column in range(3)
        )
        reconstructed_translation = [
            self.snapshot["world_to_camera_link"]["translation_m"][i]
            + matrix_vector(
                world_link_rotation,
                self.snapshot["camera_link_to_optical"]["translation_m"],
            )[i]
            for i in range(3)
        ]
        reconstruction_translation_error = math.dist(
            reconstructed_translation,
            self.snapshot["world_to_optical"]["translation_m"],
        )
        points = [
            (geometry["prim_path"], name, point)
            for geometry in self.snapshot["reference_geometry"]
            for name, point in geometry["world_surface_points_m"].items()
        ]
        measurements = []
        for reference_prim_path, name, world in points:
            link, optical = geometry_intermediates(world, self.snapshot)
            usd, optical_direct = usd_camera_direct_optical(world, self.snapshot)
            diagnostic = {
                "point": name,
                "reference_prim_path": reference_prim_path,
                **point_diagnostics(world, self.snapshot),
                "usd_camera_local_xyz": usd,
                "usd_camera_convention": "+X right, +Y up, -Z forward (UsdGeom.Camera)",
                "usd_to_ros_optical_point_coordinate_basis": self.snapshot[
                    "usd_camera_to_ros_optical_point_coordinate_basis"
                ],
                "optical_direct_xyz": optical_direct,
                "snapshot_chain_minus_direct_m": [
                    optical[i] - optical_direct[i] for i in range(3)
                ],
                "intrinsics": {
                    "fx": info.k[0],
                    "fy": info.k[4],
                    "cx": info.k[2],
                    "cy": info.k[5],
                },
            }
            if optical[2] <= 0:
                measurements.append(
                    {
                        **diagnostic,
                        "status": "NOT VERIFIED",
                        "reason": "reference point is behind camera",
                    }
                )
                continue
            u, v = project(optical, info)
            pixel = (round(u), round(v))
            if not (0 <= pixel[0] < depth.width and 0 <= pixel[1] < depth.height):
                measurements.append(
                    {
                        **diagnostic,
                        "computed_uv": [u, v],
                        "pixel": pixel,
                        "status": "NOT VERIFIED",
                        "reason": "projected pixel outside image",
                    }
                )
                continue
            actual = depth_value_at(depth, pixel[1], pixel[0], decoded=decoded_depth)
            z = optical[2]
            distance = math.sqrt(sum(value * value for value in optical))
            hypotheses = {}
            for hypothesis, candidate_pixel in depth_coordinate_hypotheses(
                depth.width, depth.height, *pixel
            ).items():
                candidate_column, candidate_row = candidate_pixel
                hypotheses[hypothesis] = {
                    "pixel": candidate_pixel,
                    "depth": depth_value_at(
                        depth, candidate_row, candidate_column, decoded=decoded_depth
                    ),
                    "patch_5x5": depth_patch_summary(
                        depth, candidate_row, candidate_column, decoded=decoded_depth
                    ),
                }
            sample_ok = math.isfinite(actual) and actual > 0
            scene_query = collision_query_entry(
                self.snapshot, reference_prim_path, name
            )
            if scene_query is not None:
                scene_query = {
                    **scene_query,
                    "pixel": list(pixel),
                    "ros_depth": actual,
                    "depth_minus_hit_z": (
                        actual - scene_query["hit_optical_z_m"]
                        if scene_query.get("hit_optical_z_m") is not None
                        else None
                    ),
                }
            measurements.append(
                {
                    **diagnostic,
                    "computed_uv": [u, v],
                    "pixel": pixel,
                    "actual_depth": actual,
                    "expected_optical_z_m": z,
                    "expected_range_m": distance,
                    "z_error_m": actual - z,
                    "range_error_m": actual - distance,
                    "depth_patch_5x5": depth_patch_summary(
                        depth, pixel[1], pixel[0], decoded=decoded_depth
                    ),
                    "coordinate_hypotheses": hypotheses,
                    "collision_scene_query": scene_query,
                    "sample_read_status": "VERIFIED" if sample_ok else "NOT VERIFIED",
                    "geometry_match_status": "NOT VERIFIED",
                }
            )
        valid = [m for m in measurements if m.get("sample_read_status") == "VERIFIED"]
        clipping = self.snapshot["projection"].get("clippingRange")
        semantic_result = (
            classify_depth_samples(
                valid,
                float(clipping[0]),
                float(clipping[1]),
                self.depth_tolerance,
                self.normalized_depth_tolerance,
            )
            if clipping is not None and len(valid) == len(measurements)
            else {
                "status": "NOT VERIFIED",
                "reason": "missing clipping range or valid depth samples",
            }
        )
        semantics = (
            semantic_result.get("semantic")
            if semantic_result["status"] == "VERIFIED"
            else None
        )
        for measurement in valid:
            measurement["geometry_match_status"] = (
                "VERIFIED" if semantics else "NOT VERIFIED"
            )
        distribution = depth_frame_distribution(depth, decoded=decoded_depth)
        hypothesis_ordering = {}
        for hypothesis in (
            "identity",
            "horizontal_flip",
            "vertical_flip",
            "both_flips",
        ):
            entries = [
                {
                    "point": measurement["point"],
                    "expected_optical_z_m": measurement["expected_optical_z_m"],
                    "observed_depth": measurement["coordinate_hypotheses"][hypothesis][
                        "depth"
                    ],
                }
                for measurement in valid
                if math.isfinite(
                    measurement["coordinate_hypotheses"][hypothesis]["depth"]
                )
            ]
            hypothesis_ordering[hypothesis] = ordering_diagnostic(entries)
        artifacts = (
            save_diagnostic_artifacts(
                self.diagnostic_output_dir, stamp, decoded_depth, rgb, measurements
            )
            if self.diagnostic_output_dir
            else None
        )
        uniform_frame = (
            distribution["finite_count"] == distribution["pixel_count"]
            and distribution["finite_value_spread"] == 0.0
        )
        self.result = {
            "pair_stamp_ns": stamp,
            "tf": tf,
            "camera_link_to_optical_pose_rotation": pose_rotation,
            "point_conversion_rotation": used_rotation,
            "point_conversion_is_pose_inverse": used_rotation
            == tuple(zip(*pose_rotation)),
            "measurements": measurements,
            "world_optical_reconstruction": {
                "translation_error_m": reconstruction_translation_error,
                "rotation_matrix_max_error": reconstruction_rotation_error,
            },
            "depth_semantic_candidates": semantic_result,
            "depth_frame_distribution": distribution,
            "depth_coarse_grid": coarse_depth_grid(decoded_depth),
            "coordinate_hypothesis_ordering": hypothesis_ordering,
            "diagnostic_artifacts": artifacts,
            "depth_distribution_diagnosis": (
                "UNIFORM_FINITE_FRAME: investigate depth generation/publisher path"
                if uniform_frame
                else "NONUNIFORM_FRAME: compare reference patches before attributing a depth generation failure"
            ),
            "depth_unit": (
                "VERIFIED: " + semantic_result["depth_unit"]
                if semantics
                else "NOT VERIFIED"
            ),
            "depth_semantics": (
                "VERIFIED: " + semantics if semantics else "NOT VERIFIED"
            ),
            "pixel_correspondence": (
                "VERIFIED"
                if semantics
                and rgb.header.frame_id == depth.header.frame_id
                and (rgb.width, rgb.height) == (depth.width, depth.height)
                else "PARTIALLY VERIFIED" if valid else "NOT VERIFIED"
            ),
        }
        self.result["status"] = (
            "VERIFIED"
            if semantics and all(entry["status"] == "VERIFIED" for entry in tf.values())
            else "NOT VERIFIED"
        )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--snapshot", required=True)
    p.add_argument("--depth-tolerance-m", type=float, required=True)
    p.add_argument("--normalized-depth-tolerance", type=float)
    p.add_argument("--tf-translation-tolerance-m", type=float, required=True)
    p.add_argument("--tf-rotation-tolerance-rad", type=float, required=True)
    p.add_argument("--diagnostic-output-dir")
    p.add_argument("--timeout-wall-s", type=float, default=20.0)
    a = p.parse_args()
    rclpy.init()
    node = Acceptance(
        json.load(open(a.snapshot)),
        a.depth_tolerance_m,
        a.normalized_depth_tolerance,
        a.tf_translation_tolerance_m,
        a.tf_rotation_tolerance_rad,
        diagnostic_output_directory(a.diagnostic_output_dir),
    )
    end = time.monotonic() + a.timeout_wall_s
    while node.result is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = node.result or {
        "status": "NOT VERIFIED",
        "reason": "wall timeout waiting for exact RGB-depth pair, CameraInfo, and TF",
    }
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["status"] == "VERIFIED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
