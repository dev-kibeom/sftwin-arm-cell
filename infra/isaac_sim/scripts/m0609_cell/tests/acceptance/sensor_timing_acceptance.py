"""Measure one DetectTarget request using live RGB-D and diagnostic topics."""

import argparse
import json
import time


RGB = "/camera/color/image_raw"
DEPTH = "/camera/aligned_depth_to_color/image_raw"
IDLE = 0
DETECTING = 1


class SensorTimingCollector:
    """Keep sensor samples only between one request's DETECTING and IDLE."""

    def __init__(self, sync_slop_ms=10.0):
        if sync_slop_ms < 0:
            raise ValueError("sync slop must be non-negative")
        self.sync_slop_ns = int(sync_slop_ms * 1_000_000)
        self.request = None
        self.terminal = None
        self.rgb = []
        self.depth = []
        self.pairs = []
        self._used = {"rgb": set(), "depth": set()}

    @property
    def complete(self):
        return self.terminal is not None

    def observe_diagnostic(
        self, state, target_id, source_frame_id, detail, receipt_s, result_code=0
    ):
        if state == DETECTING and not source_frame_id:
            if self.request is None:
                self.request = {"target_id": target_id, "start_s": receipt_s}
            return
        if state == DETECTING and source_frame_id:
            if self.request and self.request["target_id"] == target_id:
                self.request.setdefault("first_usable_diagnostic_s", receipt_s)
            return
        if state != IDLE or not self.request or self.request["target_id"] != target_id:
            return

        result_code = int(result_code)
        result_names = {
            0: "success",
            1: "object_not_found",
            2: "timeout",
            3: "sensor_error",
            4: "tf_error",
            5: "geometry_error",
            6: "invalid_result",
            7: "temporary_invalid_target",
        }
        total_ms = round((receipt_s - self.request["start_s"]) * 1000, 3)
        self.terminal = {
            "target_id": target_id,
            "result_code": result_code,
            "result": result_names.get(result_code, f"unknown_{result_code}"),
            "diagnostic_detail": detail,
            "terminal_receipt_steady_s": receipt_s,
            "request_total_elapsed_ms": total_ms,
        }

    def observe_image(self, stream, stamp_ns, receipt_s, width, height, frame_id):
        if self.request is None or self.complete:
            return False
        if (
            stream not in ("rgb", "depth")
            or not isinstance(stamp_ns, int)
            or stamp_ns < 0
        ):
            return False
        sample = {
            "stamp_ns": stamp_ns,
            "receipt_steady_s": receipt_s,
        }
        samples = self.rgb if stream == "rgb" else self.depth
        sample["request_elapsed_ms"] = round(
            (receipt_s - self.request["start_s"]) * 1000, 3
        )
        samples.append(sample)
        self._pair_for(sample, stream)
        return True

    def _pair_for(self, sample, stream):
        other_name = "depth" if stream == "rgb" else "rgb"
        other_samples = self.depth if stream == "rgb" else self.rgb
        candidates = [
            (abs(sample["stamp_ns"] - other["stamp_ns"]), index, other)
            for index, other in enumerate(other_samples)
            if index not in self._used[other_name]
            and abs(sample["stamp_ns"] - other["stamp_ns"]) <= self.sync_slop_ns
        ]
        if not candidates:
            return
        delta_ns, other_index, other = min(candidates, key=lambda item: item[0])
        current_index = len(self.rgb if stream == "rgb" else self.depth) - 1
        current_name = stream
        self._used[current_name].add(current_index)
        self._used[other_name].add(other_index)
        rgb, depth = (sample, other) if stream == "rgb" else (other, sample)
        pair_receipt_s = max(rgb["receipt_steady_s"], depth["receipt_steady_s"])
        self.pairs.append(
            {
                "rgb_stamp_ns": rgb["stamp_ns"],
                "rgb_receipt_steady_s": rgb["receipt_steady_s"],
                "depth_stamp_ns": depth["stamp_ns"],
                "depth_receipt_steady_s": depth["receipt_steady_s"],
                "delta_ms": delta_ns / 1_000_000,
                "pair_receipt_steady_s": pair_receipt_s,
                "request_elapsed_ms": round(
                    (pair_receipt_s - self.request["start_s"]) * 1000, 3
                ),
            }
        )

    def _request_deltas(self, samples):
        return [
            round(later["receipt_steady_s"] - earlier["receipt_steady_s"], 6)
            for earlier, later in zip(samples, samples[1:])
        ]

    def summary(self, elapsed_wall_s):
        start_s = self.request["start_s"] if self.request else None
        nearest = min(
            (
                (abs(rgb["stamp_ns"] - depth["stamp_ns"]), rgb, depth)
                for rgb in self.rgb
                for depth in self.depth
            ),
            key=lambda item: item[0],
            default=None,
        )
        within_slop_count = sum(
            abs(rgb["stamp_ns"] - depth["stamp_ns"]) <= self.sync_slop_ns
            for rgb in self.rgb
            for depth in self.depth
        )
        first_rgb = self.rgb[0] if self.rgb else None
        first_depth = self.depth[0] if self.depth else None
        return {
            "status": "CAPTURED" if self.terminal else "NO_TERMINAL_REQUEST",
            "request_lifecycle": {
                "captured": self.request is not None,
                "target_id": self.request["target_id"] if self.request else None,
                "start_receipt_steady_s": start_s,
                "terminal_received": self.terminal is not None,
            },
            "request_elapsed_to_first_receipt_ms": {
                "rgb": first_rgb["request_elapsed_ms"] if first_rgb else None,
                "depth": first_depth["request_elapsed_ms"] if first_depth else None,
                "synchronized_pair": (
                    self.pairs[0]["request_elapsed_ms"] if self.pairs else None
                ),
                "first_usable_observation_diagnostic": (
                    round(
                        (self.request["first_usable_diagnostic_s"] - start_s) * 1000,
                        3,
                    )
                    if self.request and "first_usable_diagnostic_s" in self.request
                    else None
                ),
            },
            "rgb": {
                "count": len(self.rgb),
                "receipt_gaps_s": self._request_deltas(self.rgb),
                "max_receipt_gap_s": max(self._request_deltas(self.rgb), default=None),
                "samples": list(self.rgb),
            },
            "depth": {
                "count": len(self.depth),
                "receipt_gaps_s": self._request_deltas(self.depth),
                "max_receipt_gap_s": max(
                    self._request_deltas(self.depth), default=None
                ),
                "samples": list(self.depth),
            },
            "rgb_depth_candidates": {
                "nearest_delta_ms": nearest[0] / 1_000_000 if nearest else None,
                "nearest_candidate": (
                    {
                        "rgb_stamp_ns": nearest[1]["stamp_ns"],
                        "rgb_receipt_steady_s": nearest[1]["receipt_steady_s"],
                        "depth_stamp_ns": nearest[2]["stamp_ns"],
                        "depth_receipt_steady_s": nearest[2]["receipt_steady_s"],
                    }
                    if nearest
                    else None
                ),
                "within_sync_slop_count": within_slop_count,
                "sync_slop_ms": self.sync_slop_ns / 1_000_000,
                "timeout_has_candidate_within_sync_slop": (
                    within_slop_count > 0
                    if self.terminal and self.terminal["result"] == "timeout"
                    else None
                ),
            },
            "synchronized_pairs": {
                "count": len(self.pairs),
                "deltas_ms": [pair["delta_ms"] for pair in self.pairs],
                "observations": list(self.pairs),
            },
            "terminal": self.terminal,
            "elapsed_wall_s": elapsed_wall_s,
        }


def acceptance_exit_code(summary):
    return 0 if summary["status"] == "CAPTURED" else 1


def _stamp_ns(stamp):
    if stamp.sec < 0 or stamp.nanosec < 0 or stamp.nanosec >= 1_000_000_000:
        return None
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--sync-slop-ms", type=float, default=10.0)
    parser.add_argument("--timeout-wall-s", type=float, default=180.0)
    args = parser.parse_args(argv)
    if args.timeout_wall_s <= 0:
        parser.error("collector startup timeout must be positive")
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from arm_cell_interfaces.msg import VisionDiagnostic
    except ModuleNotFoundError as error:
        raise SystemExit(
            "ROS 2 Python packages are unavailable; source the Humble and workspace overlays."
        ) from error

    class TimingAcceptanceNode(Node):
        def __init__(self, collector):
            super().__init__("sensor_timing_acceptance")
            self.collector = collector
            self.create_subscription(
                Image, RGB, lambda msg: self._image("rgb", msg), qos_profile_sensor_data
            )
            self.create_subscription(
                Image,
                DEPTH,
                lambda msg: self._image("depth", msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                VisionDiagnostic, "/vision/diagnostics", self._diagnostic, 10
            )

        def _image(self, stream, message):
            self.collector.observe_image(
                stream,
                _stamp_ns(message.header.stamp),
                time.monotonic(),
                message.width,
                message.height,
                message.header.frame_id,
            )

        def _diagnostic(self, message):
            self.collector.observe_diagnostic(
                message.state,
                message.target_id,
                message.source_rgb_header.frame_id,
                message.diagnostic_detail,
                time.monotonic(),
                message.result_code,
            )
            if self.collector.complete:
                self.get_logger().info("terminal DetectTarget diagnostic received")

    collector = SensorTimingCollector(args.sync_slop_ms)
    started = time.monotonic()
    rclpy.init()
    node = TimingAcceptanceNode(collector)
    try:
        deadline = started + args.timeout_wall_s
        while rclpy.ok() and not collector.complete and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        summary = collector.summary(time.monotonic() - started)
        print(json.dumps(summary, indent=2))
        return acceptance_exit_code(summary)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
