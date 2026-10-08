"""Pure display projection helpers; canonical values remain owner supplied."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable


class Freshness(str, Enum):
    CURRENT = "current"
    STALE = "stale"
    UNAVAILABLE = "unavailable"


@dataclass(frozen=True)
class FeedSnapshot:
    value: Any = None
    received_at: float | None = None
    source_stamp: float | None = None
    max_age_seconds: float | None = None
    source_age_seconds: float | None = None

    def freshness(self, now: float) -> Freshness:
        if self.received_at is None or self.value is None:
            return Freshness.UNAVAILABLE
        if self.max_age_seconds is None or self.max_age_seconds <= 0:
            return Freshness.UNAVAILABLE
        if now < self.received_at:
            return Freshness.STALE
        local_age = now - self.received_at
        if local_age > self.max_age_seconds:
            return Freshness.STALE
        if self.source_age_seconds is not None and (
            self.source_age_seconds < 0
            or self.source_age_seconds + local_age >= self.max_age_seconds
        ):
            return Freshness.STALE
        return Freshness.CURRENT


@dataclass
class HubProjection:
    """Latest owner messages and their local receipt times, with no derived state."""

    feeds: dict[str, FeedSnapshot] = field(default_factory=dict)

    def observe(
        self,
        name: str,
        value: Any,
        *,
        received_at: float,
        source_stamp: float | None = None,
        max_age_seconds: float | None = None,
        source_age_seconds: float | None = None,
    ) -> None:
        self.feeds[name] = FeedSnapshot(
            value, received_at, source_stamp, max_age_seconds, source_age_seconds
        )

    def read(self, name: str, *, now: float) -> FeedSnapshot:
        snapshot = self.feeds.get(name)
        if snapshot is None:
            return FeedSnapshot()
        # Keep stale values available for diagnosis while requiring consumers to
        # inspect freshness before rendering them as current.
        return snapshot

    def freshness(self, name: str, *, now: float) -> Freshness:
        return self.read(name, now=now).freshness(now)


def camera_overlay_state(
    *,
    image_stamp: float | None,
    image_frame: str,
    image_size: tuple[int, int],
    camera_stamp: float | None,
    camera_frame: str,
    camera_size: tuple[int, int],
    target_valid: bool,
    has_selected_target_pose: bool,
    target_stamp: float | None,
    now: float,
    max_age_seconds: float | None,
    transform_available: bool,
    project: Callable[[], tuple[float, float] | None] | None = None,
) -> tuple[str, tuple[float, float] | None]:
    """Return a marker only when source, target, calibration, and TF are usable."""
    stamps = (image_stamp, target_stamp)
    if max_age_seconds is None or max_age_seconds <= 0:
        return "unavailable", None
    if any(stamp is None for stamp in stamps):
        return "unavailable", None
    if any(now < stamp or now - stamp > max_age_seconds for stamp in stamps):
        return "stale", None
    if camera_stamp is None or now < camera_stamp:
        return "unavailable", None
    if now - camera_stamp > max_age_seconds:
        return "stale", None
    if abs(image_stamp - camera_stamp) > max_age_seconds:
        return "unavailable", None
    if not target_valid or not has_selected_target_pose:
        return "unavailable", None
    if (
        image_frame != camera_frame
        or image_size[0] <= 0
        or image_size[1] <= 0
        or image_size != camera_size
    ):
        return "unavailable", None
    if not transform_available or project is None:
        return "unavailable", None
    pixel = project()
    if pixel is None:
        return "unavailable", None
    x, y = pixel
    if not (0 <= x < image_size[0] and 0 <= y < image_size[1]):
        return "unavailable", None
    return "current", (x, y)


def vision_overlay_match(
    *,
    image_stamp: float | None,
    image_frame: str,
    diagnostic_stamp: float | None,
    diagnostic_frame: str,
    diagnostic_current: bool,
    max_delta_seconds: float,
) -> str:
    """Match Vision annotation to its original RGB observation identity."""
    if not diagnostic_current:
        return "stale"
    if (
        image_stamp is None
        or diagnostic_stamp is None
        or not image_frame
        or not diagnostic_frame
        or max_delta_seconds < 0
    ):
        return "unavailable"
    if image_frame != diagnostic_frame:
        return "unavailable"
    if abs(image_stamp - diagnostic_stamp) > max_delta_seconds:
        return "stale"
    return "current"
