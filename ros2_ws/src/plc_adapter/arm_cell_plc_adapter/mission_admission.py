"""Latest Mission admission observation for the PLC's two holding registers."""
import time
from threading import Lock


class MissionAdmission:
    def __init__(self):
        self.lock = Lock()
        self.sequence = 0
        self.available = False
        self.received_at = None

    def observe(self, message):
        with self.lock:
            self.sequence = self.sequence % 0xffff + 1
            self.available = message.data
            self.received_at = time.monotonic()

    def register_values(self):
        with self.lock:
            if self.received_at is None or time.monotonic() - self.received_at > .5:
                # A stale observation must not rearm a rejected request as a fresh FALSE would.
                return 0, False
            return self.sequence, self.available
