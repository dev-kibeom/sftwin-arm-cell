"""Select renderer-depth calibration rays from a target-free exact depth baseline."""

import argparse, json, math, sys, time
from pathlib import Path
import numpy as np

SUPPORT = Path(__file__).resolve().parents[1] / "support"
if str(SUPPORT) not in sys.path:
    sys.path.insert(0, str(SUPPORT))

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
except ModuleNotFoundError as error:
    raise SystemExit(
        "source /opt/ros/humble/setup.bash and ros2_ws/install/setup.bash before running this tool"
    ) from error
from rgbd_assertions import decode_depth_frame, stamp_nanoseconds, validate_camera_info

DEPTH, INFO = "/camera/aligned_depth_to_color/image_raw", "/camera/color/camera_info"
DESIRED_Z_M = (0.80, 1.10, 1.40)


def select_targets(
    depth, info, desired_zs, safety_margin, min_separation_px, border_px
):
    selected = []
    height, width = depth.shape
    for target_z in desired_zs:
        valid = np.isfinite(depth) & (depth > target_z + safety_margin)
        valid[:border_px, :] = False
        valid[-border_px:, :] = False
        valid[:, :border_px] = False
        valid[:, -border_px:] = False
        candidates = np.argwhere(valid)
        candidates = sorted(
            candidates, key=lambda rc: float(depth[tuple(rc)]), reverse=True
        )
        chosen = None
        for row, column in candidates:
            if all(
                math.dist((int(column), int(row)), candidate["pixel"])
                >= min_separation_px
                for candidate in selected
            ):
                chosen = (int(row), int(column))
                break
        if chosen is None:
            raise RuntimeError(
                f"no baseline pixel clears target_z={target_z} m with safety_margin={safety_margin} m and separation={min_separation_px}px"
            )
        row, column = chosen
        baseline = float(depth[row, column])
        optical = (
            (column - info.k[2]) * target_z / info.k[0],
            (row - info.k[5]) * target_z / info.k[4],
            target_z,
        )
        selected.append(
            {
                "id": f"z{int(round(target_z*100)):03d}",
                "pixel": [column, row],
                "baseline_depth_m": baseline,
                "target_optical_xyz_m": list(optical),
                "expected_front_z_m": target_z,
                "visibility_margin_m": baseline - target_z,
            }
        )
    return selected


class Planner(Node):
    def __init__(self, args):
        super().__init__("renderer_depth_target_planner")
        self.args = args
        self.info = None
        self.result = None
        self.create_subscription(
            CameraInfo, INFO, lambda m: setattr(self, "info", m), 10
        )
        self.create_subscription(Image, DEPTH, self.depth_cb, 10)

    def depth_cb(self, depth):
        if self.result is not None or self.info is None:
            return
        validate_camera_info(
            self.info, depth.header.frame_id, (depth.width, depth.height)
        )
        view = decode_depth_frame(depth)
        try:
            targets = select_targets(
                view,
                self.info,
                DESIRED_Z_M,
                self.args.safety_margin_m,
                self.args.min_pixel_separation_px,
                self.args.border_px,
            )
            self.result = {
                "schema_version": 1,
                "kind": "target_free_depth_baseline_plan",
                "frame_id": depth.header.frame_id,
                "depth_stamp_ns": stamp_nanoseconds(depth.header.stamp),
                "resolution": [depth.width, depth.height],
                "camera_info": {"k": list(self.info.k), "p": list(self.info.p)},
                "safety_margin_m": self.args.safety_margin_m,
                "min_pixel_separation_px": self.args.min_pixel_separation_px,
                "targets": targets,
            }
        except Exception as error:
            self.result = {
                "status": "NOT VERIFIED",
                "reason": f"{type(error).__name__}: {error}",
            }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", required=True)
    p.add_argument("--safety-margin-m", type=float, default=0.05)
    p.add_argument("--min-pixel-separation-px", type=int, default=160)
    p.add_argument("--border-px", type=int, default=64)
    p.add_argument("--timeout-wall-s", type=float, default=20)
    a = p.parse_args()
    rclpy.init()
    node = Planner(a)
    end = time.monotonic() + a.timeout_wall_s
    while node.result is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = node.result or {"status": "NOT VERIFIED", "reason": "wall timeout"}
    print(json.dumps(result, indent=2))
    if result.get("kind"):
        Path(a.output).write_text(json.dumps(result, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
