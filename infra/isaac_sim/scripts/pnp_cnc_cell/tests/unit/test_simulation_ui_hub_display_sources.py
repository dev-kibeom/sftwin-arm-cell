from types import SimpleNamespace

import numpy as np
import pytest
from arm_cell_simulation_ui_hub.display_sources import (
    AlignedDepthHeatmapDisplaySource,
    RGBDisplaySource,
)


def image(encoding, values, *, frame_id="camera_color_optical_frame", stamp=1.0):
    if encoding == "rgb8":
        height, width = values.shape[:2]
        payload = values.tobytes()
        step = width * 3
    else:
        height, width = values.shape
        payload = values.astype("<f4").tobytes()
        step = width * 4
    seconds = int(stamp)
    nanoseconds = round((stamp - seconds) * 1e9)
    return SimpleNamespace(
        encoding=encoding,
        width=width,
        height=height,
        step=step,
        data=payload,
        is_bigendian=False,
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds),
            frame_id=frame_id,
        ),
    )


def test_rgb_display_source_preserves_pixels_and_capture_identity():
    source = RGBDisplaySource()
    rgb = np.array([[[10, 20, 30]]], dtype=np.uint8)

    frame = source.convert(image("rgb8", rgb, stamp=2.25), received_at=12.0)

    assert frame.source_id == "rgb"
    assert frame.identity == (2.25, "camera_color_optical_frame")
    assert frame.received_at == 12.0
    assert np.array_equal(frame.rgba[0, 0], [10, 20, 30, 255])


def test_depth_heatmap_uses_fixed_metric_range_and_marks_invalid_pixels():
    source = AlignedDepthHeatmapDisplaySource()
    depth = np.array([[0.6, 1.1, 1.6, 0.0, np.nan, np.inf, 2.5]], dtype=np.float32)

    frame = source.convert(image("32FC1", depth), received_at=12.0)

    assert frame.source_id == "depth"
    assert frame.rgba.shape == (1, 7, 4)
    assert np.array_equal(frame.rgba[0, 0, :], source.NEAR_COLOR)
    assert np.array_equal(frame.rgba[0, 2, :], source.FAR_COLOR)
    assert np.array_equal(frame.rgba[0, 3, :], source.INVALID_COLOR)
    assert np.array_equal(frame.rgba[0, 4, :], source.INVALID_COLOR)
    assert np.array_equal(frame.rgba[0, 5, :], source.INVALID_COLOR)
    assert np.array_equal(frame.rgba[0, 6, :], source.FAR_COLOR)


def test_depth_heatmap_fixed_range_separates_canonical_working_depths():
    source = AlignedDepthHeatmapDisplaySource()
    depths = np.array([[0.8, 1.1, 1.4]], dtype=np.float32)

    frame = source.convert(image("32FC1", depths), received_at=12.0)

    colors = frame.rgba[0, :, :3]
    assert source.MIN_DEPTH_METERS == 0.6
    assert source.MAX_DEPTH_METERS == 1.6
    assert np.all(np.any(colors[1:] != colors[:-1], axis=1))
    luminance = colors @ np.array([0.2126, 0.7152, 0.0722])
    assert np.all(np.diff(luminance) > 0)


def test_depth_heatmap_rejects_unsupported_encoding_and_bad_payload():
    source = AlignedDepthHeatmapDisplaySource()
    unsupported = image("16UC1", np.array([[1000]], dtype=np.uint16))
    with pytest.raises(ValueError, match="32FC1"):
        source.convert(unsupported, received_at=0.0)

    malformed = image("32FC1", np.array([[1.0]], dtype=np.float32))
    malformed.step = 3
    with pytest.raises(ValueError):
        source.convert(malformed, received_at=0.0)


def test_depth_heatmap_color_for_a_distance_does_not_depend_on_frame_extrema():
    source = AlignedDepthHeatmapDisplaySource()
    first = source.convert(
        image("32FC1", np.array([[1.0]], dtype=np.float32)), received_at=1.0
    )
    second = source.convert(
        image("32FC1", np.array([[1.0, 0.2, 1.8]], dtype=np.float32)),
        received_at=2.0,
    )

    assert np.array_equal(first.rgba[0, 0], second.rgba[0, 0])


def test_depth_heatmap_honors_float32_endianness_and_row_padding():
    import struct

    source = AlignedDepthHeatmapDisplaySource()
    message = SimpleNamespace(
        encoding="32FC1",
        width=1,
        height=2,
        step=8,
        is_bigendian=True,
        data=struct.pack(">f", 0.1) + b"pad!" + struct.pack(">f", 2.0) + b"pad!",
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=3, nanosec=0),
            frame_id="camera_color_optical_frame",
        ),
    )

    frame = source.convert(message, received_at=0.0)

    assert np.array_equal(frame.rgba[0, 0], source.NEAR_COLOR)
    assert np.array_equal(frame.rgba[1, 0], source.FAR_COLOR)
