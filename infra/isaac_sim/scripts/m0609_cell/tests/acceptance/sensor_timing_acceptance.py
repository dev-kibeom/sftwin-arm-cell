"""Live exact RGB-D-CameraInfo timestamp acceptance for the Isaac 1C contract."""

import argparse
import json
import time
from collections import deque


RGB = "/camera/color/image_raw"
DEPTH = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO = "/camera/color/camera_info"
STREAMS = ("rgb", "depth", "camera_info")


class ExactTripleCollector:
    """Bounded, transport-agnostic exact timestamp association."""

    def __init__(self, required_samples=8, max_pending=32):
        if required_samples <= 0 or max_pending <= 0:
            raise ValueError("required_samples and max_pending must be positive")
        self.required_samples = required_samples
        self.max_pending = max_pending
        self.pending = {stream: {} for stream in STREAMS}
        self.pending_order = {stream: deque() for stream in STREAMS}
        self.triples = []
        self.invalid_messages = 0
        self.dropped_messages = 0

    @property
    def complete(self):
        return len(self.triples) >= self.required_samples

    def observe(self, stream, stamp_ns):
        if self.complete:
            return False
        if stream not in self.pending or not isinstance(stamp_ns, int) or stamp_ns < 0:
            self.invalid_messages += 1
            return False
        if stamp_ns in self.pending[stream]:
            return False
        self.pending[stream][stamp_ns] = stamp_ns
        self.pending_order[stream].append(stamp_ns)
        while len(self.pending[stream]) > self.max_pending:
            old_stamp = self.pending_order[stream].popleft()
            self.pending[stream].pop(old_stamp, None)
            self.dropped_messages += 1

        if all(stamp_ns in self.pending[other] for other in STREAMS):
            for other in STREAMS:
                self.pending[other].pop(stamp_ns, None)
                try:
                    self.pending_order[other].remove(stamp_ns)
                except ValueError:
                    pass
            self.triples.append(
                {
                    "rgb_stamp_ns": stamp_ns,
                    "depth_stamp_ns": stamp_ns,
                    "camera_info_stamp_ns": stamp_ns,
                    "canonical_rgb_stamp_ns": stamp_ns,
                    "spread_ns": 0,
                }
            )
            return True
        return False

    def summary(self, status, elapsed_wall_s):
        return {
            "status": status,
            "required_samples": self.required_samples,
            "observed_samples": len(self.triples),
            "triples": list(self.triples),
            "max_observed_spread_ns": max(
                (triple["spread_ns"] for triple in self.triples), default=None
            ),
            "invalid_messages": self.invalid_messages,
            "dropped_messages": self.dropped_messages,
            "elapsed_wall_s": elapsed_wall_s,
        }


def acceptance_exit_code(summary):
    return 0 if summary["status"] == "VERIFIED" else 1


def _stamp_ns(stamp):
    if stamp.sec < 0 or stamp.nanosec < 0 or stamp.nanosec >= 1_000_000_000:
        return None
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--max-pending", type=int, default=32)
    parser.add_argument("--timeout-wall-s", type=float, default=20.0)
    args = parser.parse_args(argv)
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image
    except ModuleNotFoundError as error:
        raise SystemExit(
            "ROS 2 Python packages are unavailable; source the Humble and workspace overlays."
        ) from error

    class TimingAcceptanceNode(Node):
        def __init__(self, collector):
            super().__init__("sensor_timing_acceptance")
            self.collector = collector
            qos = 10
            self.create_subscription(
                Image, RGB, lambda message: self._observe("rgb", message), qos
            )
            self.create_subscription(
                Image, DEPTH, lambda message: self._observe("depth", message), qos
            )
            self.create_subscription(
                CameraInfo,
                CAMERA_INFO,
                lambda message: self._observe("camera_info", message),
                qos,
            )

        def _observe(self, stream, message):
            stamp = _stamp_ns(message.header.stamp)
            self.collector.observe(stream, stamp)

    collector = ExactTripleCollector(args.samples, args.max_pending)
    started = time.monotonic()
    rclpy.init()
    node = TimingAcceptanceNode(collector)
    try:
        deadline = started + args.timeout_wall_s
        while rclpy.ok() and not collector.complete and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        status = (
            "VERIFIED"
            if collector.complete and not collector.invalid_messages
            else "NOT VERIFIED"
        )
        summary = collector.summary(status, time.monotonic() - started)
        print(json.dumps(summary, indent=2))
        return acceptance_exit_code(summary)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
