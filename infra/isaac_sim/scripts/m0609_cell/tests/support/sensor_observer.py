"""State-only observer helpers; wall deadlines use monotonic receipt time."""

import time

from rgbd_assertions import ExactPairer, stamp_nanoseconds


class SensorObserver:
    def __init__(self, max_pending_pairs, monotonic=time.monotonic):
        self.pairer = ExactPairer(max_pending_pairs)
        self.monotonic = monotonic
        self.receipts = []
        self.clock_stamp_ns = None

    def receive_color(self, image):
        self.receipts.append(
            ("color", stamp_nanoseconds(image.header.stamp), self.monotonic())
        )
        return self.pairer.add_color(image)

    def receive_depth(self, image):
        self.receipts.append(
            ("depth", stamp_nanoseconds(image.header.stamp), self.monotonic())
        )
        return self.pairer.add_depth(image)

    def receive_clock(self, clock):
        self.clock_stamp_ns = stamp_nanoseconds(clock.clock)

    def receipt_is_fresh(self, receipt_time, maximum_age_wall_s):
        return self.monotonic() - receipt_time <= maximum_age_wall_s

    def deadline_expired(self, started_at, timeout_wall_s):
        return self.monotonic() - started_at >= timeout_wall_s
