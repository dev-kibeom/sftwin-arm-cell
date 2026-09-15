"""Classify live depth samples only against independent geometry expectations."""

import math


def _depth_buffer_value(distance, near, far):
    """Return the conventional [0, 1] perspective depth-buffer value."""
    return far * (distance - near) / (distance * (far - near))


def classify_depth_samples(
    samples, near_clip_m, far_clip_m, metric_tolerance_m, normalized_tolerance=None
):
    """Return candidate residuals without treating a 32FC1 payload as metres.

    A verdict requires three samples at independently different optical depths.
    This prevents a single planar surface from accidentally identifying image-plane
    distance, Euclidean range, normalized depth, or a depth-buffer value.
    """
    if metric_tolerance_m <= 0.0:
        raise ValueError("metric depth tolerance must be positive")
    if normalized_tolerance is not None and normalized_tolerance <= 0.0:
        raise ValueError("normalized depth tolerance must be positive")
    if not 0.0 < near_clip_m < far_clip_m:
        raise ValueError("invalid clipping range")
    if len(samples) < 3:
        return {
            "status": "NOT VERIFIED",
            "reason": "need at least three independent depth samples",
        }
    fields = ("actual_depth", "expected_optical_z_m", "expected_range_m")
    if any(
        not all(
            math.isfinite(sample[field]) and sample[field] > 0.0 for field in fields
        )
        for sample in samples
    ):
        return {
            "status": "NOT VERIFIED",
            "reason": "samples contain invalid depth or geometry values",
        }
    optical_depths = [sample["expected_optical_z_m"] for sample in samples]
    if max(optical_depths) - min(optical_depths) <= metric_tolerance_m:
        return {
            "status": "NOT VERIFIED",
            "reason": "reference samples do not span independent optical depths",
        }

    candidates = {
        "optical_axis_z_m": lambda sample: sample["expected_optical_z_m"],
        "euclidean_range_m": lambda sample: sample["expected_range_m"],
        "normalized_linear_optical_z": lambda sample: (
            sample["expected_optical_z_m"] - near_clip_m
        )
        / (far_clip_m - near_clip_m),
        "normalized_linear_range": lambda sample: (
            sample["expected_range_m"] - near_clip_m
        )
        / (far_clip_m - near_clip_m),
        "perspective_depth_buffer_optical_z": lambda sample: _depth_buffer_value(
            sample["expected_optical_z_m"], near_clip_m, far_clip_m
        ),
        "perspective_depth_buffer_range": lambda sample: _depth_buffer_value(
            sample["expected_range_m"], near_clip_m, far_clip_m
        ),
    }
    residuals = {
        name: max(abs(sample["actual_depth"] - expected(sample)) for sample in samples)
        for name, expected in candidates.items()
    }
    metric_candidates = {"optical_axis_z_m", "euclidean_range_m"}
    matches = [
        name for name in metric_candidates if residuals[name] <= metric_tolerance_m
    ]
    if normalized_tolerance is not None:
        matches.extend(
            name
            for name in candidates
            if name not in metric_candidates and residuals[name] <= normalized_tolerance
        )
    if len(matches) != 1:
        return {
            "status": "NOT VERIFIED",
            "candidate_max_error": residuals,
            "reason": (
                "no candidate matches"
                if not matches
                else "candidate semantics are ambiguous"
            ),
            "normalized_tolerance_configured": normalized_tolerance is not None,
        }
    semantic = matches[0]
    return {
        "status": "VERIFIED",
        "semantic": semantic,
        "candidate_max_error": residuals,
        "depth_unit": "metres" if semantic.endswith("_m") else "normalized",
    }
