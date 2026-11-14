"""Exact-pair observer for temporary renderer-visible optical-Z calibration targets."""

import argparse, json, math, sys, time
from pathlib import Path

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
from rgbd_assertions import (
    decode_depth_frame,
    depth_value_at,
    stamp_nanoseconds,
    validate_camera_info,
    validate_image_payload,
)

RGB, DEPTH, INFO = (
    "/camera/color/image_raw",
    "/camera/aligned_depth_to_color/image_raw",
    "/camera/color/camera_info",
)


def project(point, info):
    return (
        info.k[0] * point[0] / point[2] + info.k[2],
        info.k[4] * point[1] / point[2] + info.k[5],
    )


class Observer(Node):
    def __init__(self, manifest, tolerance):
        super().__init__("renderer_depth_calibration_acceptance")
        self.manifest, self.tolerance = manifest, tolerance
        self.info = {}
        self.rgb = {}
        self.depth = {}
        self.result = None
        self.create_subscription(CameraInfo, INFO, self.info_cb, 10)
        self.create_subscription(Image, RGB, self.rgb_cb, 10)
        self.create_subscription(Image, DEPTH, self.depth_cb, 10)

    def rgb_cb(self, m):
        self.rgb[stamp_nanoseconds(m.header.stamp)] = m
        self.try_pair(m)

    def info_cb(self, m):
        self.info[stamp_nanoseconds(m.header.stamp)] = m

    def depth_cb(self, m):
        self.depth[stamp_nanoseconds(m.header.stamp)] = m
        self.try_pair(m)

    def try_pair(self, m):
        stamp = stamp_nanoseconds(m.header.stamp)
        if (
            self.result is not None
            or stamp not in self.info
            or stamp not in self.rgb
            or stamp not in self.depth
        ):
            return
        rgb, depth, info = self.rgb[stamp], self.depth[stamp], self.info[stamp]
        validate_image_payload(rgb)
        validate_camera_info(info, rgb.header.frame_id, (rgb.width, rgb.height))
        view = decode_depth_frame(depth)
        rgb_view = memoryview(rgb.data)
        entries = []
        for target in self.manifest["targets"]:
            point = target["front_optical_xyz_m"]
            u, v = project(point, info)
            pixel = (round(u), round(v))
            if not (0 <= pixel[0] < depth.width and 0 <= pixel[1] < depth.height):
                entries.append(
                    {
                        "id": target["id"],
                        "computed_uv": [u, v],
                        "status": "NOT VERIFIED",
                        "reason": "projected pixel outside image",
                    }
                )
                continue
            observed = depth_value_at(depth, pixel[1], pixel[0], decoded=view)
            offset = pixel[1] * rgb.step + pixel[0] * 3
            rgb_sample = list(rgb_view[offset : offset + 3])
            error = observed - point[2]
            entries.append(
                {
                    "id": target["id"],
                    "pixel": list(pixel),
                    "computed_uv": [u, v],
                    "front_optical_z_m": point[2],
                    "ros_depth_m": observed,
                    "depth_minus_front_z_m": error,
                    "rgb_sample": rgb_sample,
                    "status": (
                        "VERIFIED"
                        if math.isfinite(observed) and abs(error) <= self.tolerance
                        else "NOT VERIFIED"
                    ),
                }
            )
        passed = len(entries) == 3 and all(e["status"] == "VERIFIED" for e in entries)
        self.result = {
            "pair_stamp_ns": stamp,
            "targets": entries,
            "depth_unit": "VERIFIED: metres" if passed else "NOT VERIFIED",
            "depth_semantics": (
                "VERIFIED: DistanceToImagePlane / optical-axis Z"
                if passed
                else "NOT VERIFIED"
            ),
            "pixel_correspondence": "VERIFIED" if passed else "NOT VERIFIED",
            "status": "VERIFIED" if passed else "NOT VERIFIED",
        }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--manifest", required=True)
    p.add_argument("--depth-tolerance-m", type=float, required=True)
    p.add_argument("--timeout-wall-s", type=float, default=20)
    a = p.parse_args()
    rclpy.init()
    node = Observer(json.load(open(a.manifest)), a.depth_tolerance_m)
    end = time.monotonic() + a.timeout_wall_s
    while node.result is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = node.result or {"status": "NOT VERIFIED", "reason": "wall timeout"}
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["status"] == "VERIFIED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
