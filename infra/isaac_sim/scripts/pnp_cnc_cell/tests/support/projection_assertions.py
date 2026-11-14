"""Projection assertions deliberately require declared depth semantics."""

import math


def deproject_optical_z(u, v, depth_m, camera_info):
    if not math.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError("invalid depth pixel")
    fx, fy, cx, cy = (
        camera_info.k[0],
        camera_info.k[4],
        camera_info.k[2],
        camera_info.k[5],
    )
    if not all(math.isfinite(value) and value != 0.0 for value in (fx, fy)):
        raise ValueError("invalid focal lengths")
    return ((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m)


def require_projection_contract(config):
    absent = [
        key
        for key in (
            "depth_unit",
            "depth_semantics",
            "projection_tolerance_px",
            "depth_tolerance_m",
        )
        if config.get(key) is None
    ]
    if absent:
        raise RuntimeError("NOT VERIFIED projection contract: " + ", ".join(absent))
