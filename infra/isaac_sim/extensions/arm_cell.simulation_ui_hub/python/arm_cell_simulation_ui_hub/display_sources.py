"""Small, Hub-local adapters from raw camera messages to operator frames."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class DisplayFrame:
    """RGBA presentation pixels and the identity of their camera capture."""

    source_id: str
    rgba: Any
    identity: tuple[float, str]
    received_at: float

    @property
    def width(self) -> int:
        return int(self.rgba.shape[1])

    @property
    def height(self) -> int:
        return int(self.rgba.shape[0])


class DisplaySource(Protocol):
    """Convert one source message into display-only RGBA pixels."""

    source_id: str

    def convert(self, message: Any, *, received_at: float) -> DisplayFrame: ...


def image_identity(message: Any) -> tuple[float, str]:
    stamp = message.header.stamp
    return (
        float(stamp.sec) + float(stamp.nanosec) / 1e9,
        str(message.header.frame_id),
    )


class RGBDisplaySource:
    source_id = "rgb"

    @staticmethod
    def validate(message: Any) -> None:
        width, height, step = int(message.width), int(message.height), int(message.step)
        if width <= 0 or height <= 0 or message.encoding.lower() != "rgb8":
            raise ValueError("expected a non-empty rgb8 image")
        row_bytes = width * 3
        if step < row_bytes:
            raise ValueError("rgb8 row step is smaller than its pixel data")
        if len(message.data) < height * step:
            raise ValueError("rgb8 payload is shorter than its declared dimensions")

    def convert(self, message: Any, *, received_at: float) -> DisplayFrame:
        import numpy as np

        self.validate(message)
        width, height, step = int(message.width), int(message.height), int(message.step)
        row_bytes = width * 3
        raw = np.frombuffer(message.data, dtype=np.uint8)
        rgb = (
            raw[: height * step]
            .reshape(height, step)[:, :row_bytes]
            .reshape(height, width, 3)
        )
        rgba = np.empty((height, width, 4), dtype=np.uint8)
        rgba[:, :, :3] = rgb
        rgba[:, :, 3] = 255
        return DisplayFrame(self.source_id, rgba, image_identity(message), received_at)


class AlignedDepthHeatmapDisplaySource:
    """Render metric 32FC1 aligned depth with a fixed operator-view range."""

    source_id = "depth"
    MIN_DEPTH_METERS = 0.6
    MAX_DEPTH_METERS = 1.6
    COLOR_STOPS = (
        (18, 59, 91),
        (25, 133, 137),
        (91, 186, 137),
        (216, 230, 92),
        (255, 242, 166),
    )
    NEAR_COLOR = (*COLOR_STOPS[0], 255)
    FAR_COLOR = (*COLOR_STOPS[-1], 255)
    INVALID_COLOR = (255, 0, 255, 255)

    def __init__(self) -> None:
        self._color_lut: Any | None = None

    def _get_color_lut(self) -> Any:
        if self._color_lut is None:
            import numpy as np

            normalized = np.linspace(0.0, 1.0, 256, dtype=np.float32)
            stops = np.asarray(self.COLOR_STOPS, dtype=np.float32)
            stop_positions = np.linspace(0.0, 1.0, len(stops), dtype=np.float32)
            self._color_lut = np.empty((256, 4), dtype=np.uint8)
            for channel in range(3):
                self._color_lut[:, channel] = np.rint(
                    np.interp(normalized, stop_positions, stops[:, channel])
                ).astype(np.uint8)
            self._color_lut[:, 3] = 255
            self._color_lut[0] = self.NEAR_COLOR
            self._color_lut[-1] = self.FAR_COLOR
        return self._color_lut

    @staticmethod
    def _depth_view(message: Any) -> Any:
        import numpy as np

        width, height, step = int(message.width), int(message.height), int(message.step)
        if width <= 0 or height <= 0 or message.encoding != "32FC1":
            raise ValueError("expected a non-empty 32FC1 aligned depth image")
        if step < width * 4:
            raise ValueError("32FC1 row step is smaller than its pixel data")
        if len(message.data) < height * step:
            raise ValueError("32FC1 payload is shorter than its declared dimensions")
        dtype = np.dtype(">f4" if bool(message.is_bigendian) else "<f4")
        return np.ndarray(
            shape=(height, width),
            dtype=dtype,
            buffer=memoryview(message.data),
            strides=(step, dtype.itemsize),
        )

    def convert(self, message: Any, *, received_at: float) -> DisplayFrame:
        import numpy as np

        depth = self._depth_view(message)
        valid = np.empty(depth.shape, dtype=bool)
        np.greater(depth, 0.0, out=valid)
        finite = np.isfinite(depth)
        np.logical_and(valid, finite, out=valid)
        normalized = np.zeros(depth.shape, dtype=np.float32)
        np.subtract(depth, self.MIN_DEPTH_METERS, out=normalized, where=valid)
        np.divide(
            normalized,
            self.MAX_DEPTH_METERS - self.MIN_DEPTH_METERS,
            out=normalized,
            where=valid,
        )
        np.clip(normalized, 0.0, 1.0, out=normalized)
        np.multiply(normalized, 255.0, out=normalized)
        rgba = self._get_color_lut()[normalized.astype(np.uint8)]
        rgba[~valid] = self.INVALID_COLOR
        return DisplayFrame(self.source_id, rgba, image_identity(message), received_at)
