"""Private RGB-D source-side acceptance stimulus helpers."""

from __future__ import annotations

import copy


def make_stale_sample(message, *, offset_seconds: float = 2.0):
    """Copy a real source frame and move its header stamp backwards."""
    if offset_seconds <= 0:
        raise ValueError("stale stimulus offset must be positive")
    sample = copy.deepcopy(message)
    stamp = sample.header.stamp
    total_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    stale_ns = max(0, total_ns - int(offset_seconds * 1_000_000_000))
    stamp.sec, stamp.nanosec = divmod(stale_ns, 1_000_000_000)
    return sample


class RgbdStaleStimulus:
    """Replay a source-owned camera frame with an old stamp on its data topic.

    This helper is constructed only by the RGB-D source scenario port. It uses
    the existing production image topic and creates no control topic or Hub
    publisher. The normal CameraHelper is disabled by the caller after a real
    source sample has been captured, so there is one source at a time.
    """

    def __init__(self, topic: str, *, offset_seconds: float = 2.0):
        self.topic = topic
        self.offset_seconds = offset_seconds
        self.node = None
        self.executor = None
        self.thread = None
        self._sample = None
        self._closed = False

    def start(self) -> bool:
        import threading
        import time

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node("rgbd_source_stale_stimulus")
        publisher = self.node.create_publisher(
            Image, self.topic, qos_profile_sensor_data
        )

        def receive(message):
            stamp = message.header.stamp
            sample_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if self._sample is None or sample_ns > self._sample[0]:
                stale = make_stale_sample(message, offset_seconds=self.offset_seconds)
                self._sample = (
                    int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
                    stale,
                )

        self.node.create_subscription(
            Image, self.topic, receive, qos_profile_sensor_data
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        deadline = time.monotonic() + 2.0
        while self._sample is None and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        if self._sample is None:
            self.close()
            return False

        def run():
            next_publish = 0.0
            while not self._closed:
                self.executor.spin_once(timeout_sec=0.02)
                now = time.monotonic()
                if now >= next_publish and self._sample is not None:
                    publisher.publish(copy.deepcopy(self._sample[1]))
                    next_publish = now + 0.1

        self.thread = threading.Thread(
            target=run, name="rgbd-source-stale-stimulus", daemon=True
        )
        self.thread.start()
        return True

    def close(self) -> None:
        self._closed = True
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        if self.executor is not None:
            try:
                self.executor.remove_node(self.node)
                self.executor.shutdown()
            except (AttributeError, RuntimeError):
                pass
            self.executor = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
