"""Owner-local Holding Uncertainty stimulus for the simulated backend port."""

from __future__ import annotations

import threading
import time

_lock = threading.Lock()
_holding_unknown_active = False


def set_holding_unknown(active: bool) -> bool:
    """Activate/clear the approved private observation-boundary stimulus."""
    global _holding_unknown_active
    with _lock:
        _holding_unknown_active = bool(active)
    return True


def holding_unknown_active() -> bool:
    with _lock:
        return _holding_unknown_active


def observe_with_scenario(observation, *, unknown_factory, now=None):
    """Overlay UNKNOWN at the backend observation port without state writes."""
    if not holding_unknown_active():
        return observation
    now = time.monotonic() if now is None else now
    return unknown_factory(now, max(1, observation.sequence))
