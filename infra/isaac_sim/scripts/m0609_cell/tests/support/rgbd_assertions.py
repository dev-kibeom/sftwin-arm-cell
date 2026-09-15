"""Pure RGB-D transport assertions used by bag and live observers."""

import math

import numpy as np


SUPPORTED_ENCODINGS = {"rgb8": 3, "32FC1": 4}


def stamp_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_image_payload(image):
    channels_or_bytes = SUPPORTED_ENCODINGS.get(image.encoding)
    if channels_or_bytes is None:
        raise ValueError(f"unsupported encoding: {image.encoding}")
    minimum_step = image.width * channels_or_bytes
    if image.step < minimum_step:
        raise ValueError("image step is shorter than a row")
    expected_length = image.step * image.height
    if len(image.data) != expected_length:
        raise ValueError("image data length does not match step * height")
    return {
        "row_padding_bytes": image.step - minimum_step,
        "byte_order": "big" if image.is_bigendian else "little",
    }


def decode_depth_frame(image):
    """Return a zero-copy, strided float32 view of one validated depth frame."""
    if image.encoding != "32FC1":
        raise ValueError("depth must use 32FC1")
    validate_image_payload(image)
    dtype = np.dtype(">f4" if image.is_bigendian else "<f4")
    return np.ndarray(
        shape=(image.height, image.width),
        dtype=dtype,
        buffer=memoryview(image.data),
        strides=(image.step, dtype.itemsize),
    )


def depth_value_at(image, row, column, decoded=None):
    view = decode_depth_frame(image) if decoded is None else decoded
    if not (0 <= row < image.height and 0 <= column < image.width):
        raise ValueError("depth pixel is outside the image")
    return float(view[row, column])


def is_valid_depth(value):
    return math.isfinite(value) and value > 0.0


def depth_patch_summary(image, row, column, radius=2, decoded=None):
    """Describe a bounded depth patch without replacing an invalid centre pixel."""
    if not (0 <= row < image.height and 0 <= column < image.width):
        raise ValueError("depth patch centre is outside the image")
    view = decode_depth_frame(image) if decoded is None else decoded
    patch = view[
        max(0, row - radius) : min(image.height, row + radius + 1),
        max(0, column - radius) : min(image.width, column + radius + 1),
    ]
    finite = patch[np.isfinite(patch)]
    return {
        "radius_px": radius,
        "sample_count": int(patch.size),
        "finite_count": int(finite.size),
        "min": float(np.min(finite)) if finite.size else None,
        "max": float(np.max(finite)) if finite.size else None,
        "center": float(view[row, column]),
    }


def depth_frame_distribution(image, unique_cap=10000, decoded=None):
    """Return full-frame descriptive statistics; values retain their published units."""
    view = decode_depth_frame(image) if decoded is None else decoded
    finite = view[np.isfinite(view)]
    unique = np.unique(finite)
    samples = {
        "center": [image.width // 2, image.height // 2],
        "top_left": [0, 0],
        "top_right": [image.width - 1, 0],
        "bottom_left": [0, image.height - 1],
        "bottom_right": [image.width - 1, image.height - 1],
    }
    finite_max = float(np.max(finite)) if finite.size else None
    nonmax_mask = (
        np.isfinite(view) & (view < finite_max)
        if finite.size
        else np.zeros(view.shape, dtype=bool)
    )
    rows, columns = np.nonzero(nonmax_mask)
    return {
        "pixel_count": image.width * image.height,
        "finite_count": int(finite.size),
        "positive_infinity_count": int(np.count_nonzero(np.isposinf(view))),
        "negative_infinity_count": int(np.count_nonzero(np.isneginf(view))),
        "nan_count": int(np.count_nonzero(np.isnan(view))),
        "min": float(np.min(finite)) if finite.size else None,
        "max": finite_max,
        "mean": float(np.mean(finite)) if finite.size else None,
        "percentiles": {
            f"p{percent}": (
                float(np.percentile(finite, percent)) if finite.size else None
            )
            for percent in (1, 10, 50, 90, 99)
        },
        "finite_value_spread": float(np.ptp(finite)) if finite.size else None,
        "distinct_finite_count_capped": int(min(unique.size, unique_cap)),
        "distinct_cap_reached": unique.size >= unique_cap,
        "finite_nonmax": {
            "count": int(np.count_nonzero(nonmax_mask)),
            "fraction_of_finite": (
                float(np.count_nonzero(nonmax_mask) / finite.size)
                if finite.size
                else None
            ),
            "image_bbox_xyxy": (
                [
                    int(columns.min()),
                    int(rows.min()),
                    int(columns.max()),
                    int(rows.max()),
                ]
                if rows.size
                else None
            ),
        },
        "fixed_samples": {
            name: depth_value_at(image, point[1], point[0], decoded=view)
            for name, point in samples.items()
        },
    }


def coarse_depth_grid(decoded, columns=16, rows=9):
    """Summarize finite and non-maximum depth locations without inferring units."""
    if columns <= 0 or rows <= 0:
        raise ValueError("coarse grid dimensions must be positive")
    height, width = decoded.shape
    finite = decoded[np.isfinite(decoded)]
    finite_max = float(np.max(finite)) if finite.size else None
    cells = []
    for grid_y in range(rows):
        y0, y1 = height * grid_y // rows, height * (grid_y + 1) // rows
        for grid_x in range(columns):
            x0, x1 = width * grid_x // columns, width * (grid_x + 1) // columns
            tile = decoded[y0:y1, x0:x1]
            finite_tile = tile[np.isfinite(tile)]
            cells.append(
                {
                    "grid_xy": [grid_x, grid_y],
                    "image_bounds_xyxy": [x0, y0, x1 - 1, y1 - 1],
                    "finite_count": int(finite_tile.size),
                    "positive_infinity_count": int(np.count_nonzero(np.isposinf(tile))),
                    "finite_nonmax_count": (
                        int(np.count_nonzero(np.isfinite(tile) & (tile < finite_max)))
                        if finite_max is not None
                        else 0
                    ),
                    "min": float(np.min(finite_tile)) if finite_tile.size else None,
                    "max": float(np.max(finite_tile)) if finite_tile.size else None,
                }
            )
    return {"columns": columns, "rows": rows, "cells": cells}


def depth_coordinate_hypotheses(width, height, column, row):
    """Return only dimension-preserving raster-orientation alternatives."""
    return {
        "identity": [column, row],
        "horizontal_flip": [width - 1 - column, row],
        "vertical_flip": [column, height - 1 - row],
        "both_flips": [width - 1 - column, height - 1 - row],
    }


def validate_camera_info(info, expected_frame, expected_resolution):
    if info.header.frame_id != expected_frame:
        raise ValueError("CameraInfo frame differs from image frame")
    if (info.width, info.height) != tuple(expected_resolution):
        raise ValueError("CameraInfo resolution differs from image resolution")
    values = list(info.k) + list(info.p) + list(info.r) + list(info.d)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("CameraInfo contains non-finite calibration values")
    if not info.distortion_model:
        raise ValueError("CameraInfo distortion model is empty")


class ExactPairer:
    """Pairs independent arrivals only when their integer ROS stamps are equal."""

    def __init__(self, max_pending):
        self.max_pending = max_pending
        self._colors = {}
        self._depths = {}
        self.pairs = []
        self.duplicates = 0

    def add_color(self, message):
        return self._add(self._colors, self._depths, message, "color")

    def add_depth(self, message):
        return self._add(self._depths, self._colors, message, "depth")

    def _add(self, own, other, message, kind):
        stamp = stamp_nanoseconds(message.header.stamp)
        if stamp in own:
            self.duplicates += 1
            raise ValueError(f"duplicate {kind} stamp: {stamp}")
        counterpart = other.pop(stamp, None)
        if counterpart is not None:
            pair = (message, counterpart) if kind == "color" else (counterpart, message)
            self.pairs.append(pair)
            return pair
        own[stamp] = message
        if len(own) > self.max_pending:
            raise ValueError("pending exact-pair bound exceeded")
        return None

    @property
    def unmatched_count(self):
        return len(self._colors) + len(self._depths)
