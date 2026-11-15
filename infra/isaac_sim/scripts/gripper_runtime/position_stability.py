"""Bounded position variation over a continuous observation interval."""
import math


class PositionStability:
    def __init__(self):
        self.reset()

    def reset(self):
        self.minimum = self.maximum = None
        self.duration = 0.0

    def update(self, position, dt, tolerance):
        if not all(math.isfinite(v) for v in (position, dt, tolerance)) or dt <= 0 or tolerance <= 0:
            self.reset()
            return 0.0
        continuing = self.minimum is not None
        low = position if self.minimum is None else min(position, self.minimum)
        high = position if self.maximum is None else max(position, self.maximum)
        if high - low > tolerance:
            self.reset()
            low = high = position
            continuing = False
        self.minimum, self.maximum = low, high
        if continuing:
            self.duration += dt
        return self.duration
