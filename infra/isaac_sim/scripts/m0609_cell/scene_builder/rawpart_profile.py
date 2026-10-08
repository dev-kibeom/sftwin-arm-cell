"""Shared operational RawPart profile loading and visible fiducial geometry."""

import math
from pathlib import Path

import cv2
import yaml


PROFILE_RELATIVE_PATH = Path(
    "ros2_ws/src/arm_cell/arm_cell_bringup/config/operational_profile.yaml"
)


def load_rawpart_profile(project_root):
    """Load and validate the RawPart deployment values from the ROS profile."""
    path = Path(project_root).expanduser().resolve() / PROFILE_RELATIVE_PATH
    if not path.is_file():
        raise ValueError(f"operational profile is unavailable: {path}")
    config = yaml.safe_load(path.read_text()) or {}
    profile = config.get("vision", {}).get("detector_profiles", {}).get("RawPart")
    if not isinstance(profile, dict) or profile.get("target_id") != "RawPart":
        raise ValueError("Vision must configure the RawPart target profile")
    if profile.get("shape") != "box":
        raise ValueError("RawPart profile shape must be box")
    _vector(profile, "dimensions_m", 3, positive=True)
    _vector(profile, "dimension_tolerance_m", 3, nonnegative=True)
    _vector(profile.get("workspace_roi_m", {}), "x", 2)
    _vector(profile.get("workspace_roi_m", {}), "y", 2)
    _vector(profile.get("workspace_roi_m", {}), "z", 2)
    for axis in ("x", "y", "z"):
        low, high = profile["workspace_roi_m"][axis]
        if low >= high:
            raise ValueError(f"RawPart workspace ROI {axis} must be increasing")
    for key in ("support_contact_tolerance_m", "top_surface_geometry_tolerance_m"):
        if not _positive_finite(profile.get(key)):
            raise ValueError(f"RawPart profile {key} must be positive and finite")
    support = _vector(profile, "support_region_dimensions_m", 2, positive=True)
    fiducial = profile.get("fiducial")
    if not isinstance(fiducial, dict):
        raise ValueError("RawPart profile fiducial is required")
    if fiducial.get("support_frame") != "RawSlot":
        raise ValueError("RawPart fiducial support_frame must be RawSlot")
    relation = fiducial.get("marker_in_support")
    if not isinstance(relation, dict):
        raise ValueError("RawPart fiducial marker_in_support relation is required")
    position = _vector(relation, "position_m", 3)
    _vector(relation, "rpy_rad", 3)
    size = fiducial.get("size_m")
    if not _positive_finite(size):
        raise ValueError("RawPart fiducial size_m must be positive and finite")
    if not all(
        abs(position[axis]) + size / 2.0 <= support[axis] / 2.0 for axis in (0, 1)
    ):
        raise ValueError("RawPart fiducial marker_in_support lies outside RawSlot")
    dictionary_name = fiducial.get("dictionary")
    if not isinstance(dictionary_name, str) or not hasattr(cv2.aruco, dictionary_name):
        raise ValueError("RawPart fiducial dictionary is unsupported")
    marker_id = fiducial.get("marker_id")
    if type(marker_id) is not int or marker_id < 0:
        raise ValueError("RawPart fiducial marker_id must be a non-negative integer")
    _validate_rotation(relation["rpy_rad"])
    return profile


def _vector(mapping, name, length, *, positive=False, nonnegative=False):
    values = mapping.get(name) if isinstance(mapping, dict) else None
    if (
        not isinstance(values, (list, tuple))
        or len(values) != length
        or not all(_finite(value) for value in values)
    ):
        raise ValueError(f"RawPart profile {name} must contain {length} finite values")
    if positive and not all(value > 0 for value in values):
        raise ValueError(f"RawPart profile {name} values must be positive")
    if nonnegative and not all(value >= 0 for value in values):
        raise ValueError(f"RawPart profile {name} values must be non-negative")
    return tuple(float(value) for value in values)


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def _positive_finite(value):
    return _finite(value) and value > 0


def _validate_rotation(rpy):
    if (
        not isinstance(rpy, (list, tuple))
        or len(rpy) != 3
        or not all(_finite(value) for value in rpy)
    ):
        raise ValueError("RawPart fiducial marker_in_support.rpy_rad must be finite")


def marker_black_cells(profile):
    """Return black ArUco grid cells generated from the profile dictionary and ID."""
    fiducial = profile["fiducial"]
    dictionary_id = getattr(cv2.aruco, fiducial["dictionary"])
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    generator = getattr(cv2.aruco, "generateImageMarker", None)
    if generator is not None:
        marker = generator(dictionary, fiducial["marker_id"], 60, borderBits=1)
    else:
        marker = cv2.aruco.drawMarker(
            dictionary, fiducial["marker_id"], 60, borderBits=1
        )
    return tuple(
        (row, column)
        for row in range(6)
        for column in range(6)
        if marker[row * 10 + 5, column * 10 + 5] < 128
    )


def marker_cell_quads(profile):
    """Return each marker square's four corners in the RawSlot support frame."""
    fiducial = profile["fiducial"]
    relation = fiducial["marker_in_support"]
    size = fiducial["size_m"]
    cell_size = size / 6.0
    roll, pitch, yaw = (float(value) for value in relation["rpy_rad"])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    tx, ty, tz = (float(value) for value in relation["position_m"])
    black = set(marker_black_cells(profile))
    quads = []
    for row in range(6):
        for column in range(6):
            x0 = -size / 2.0 + column * cell_size
            x1 = x0 + cell_size
            y1 = size / 2.0 - row * cell_size
            y0 = y1 - cell_size
            # Counter-clockwise from above gives the tile a +Z support normal.
            local_corners = ((x0, y1, 0.0), (x0, y0, 0.0), (x1, y0, 0.0), (x1, y1, 0.0))
            corners = tuple(
                (
                    tx + sum(rotation[0][i] * point[i] for i in range(3)),
                    ty + sum(rotation[1][i] * point[i] for i in range(3)),
                    tz + sum(rotation[2][i] * point[i] for i in range(3)),
                )
                for point in local_corners
            )
            quads.append((row, column, corners, (row, column) in black))
    return tuple(quads)


def author_support_fiducial(context, profile, support_origin_world):
    """Author the configured ArUco as non-colliding, sensor-visible USD faces."""
    from pxr import Gf, UsdGeom, Vt

    profile_fiducial = profile["fiducial"]
    if profile_fiducial["support_frame"] != "RawSlot":
        raise ValueError("RawPart scene fiducial must use the RawSlot support frame")
    origin = tuple(float(value) for value in support_origin_world)
    mesh = UsdGeom.Mesh.Define(
        context.stage,
        f"{context.cell_root}/AMR/Mockup/RawSlotSupportFiducial",
    )
    points = []
    indices = []
    colors = []
    for _row, _column, corners, is_black in marker_cell_quads(profile):
        for x, y, z in corners:
            points.append(Gf.Vec3f(origin[0] + x, origin[1] + y, origin[2] + z))
        indices.extend(range(len(indices), len(indices) + 4))
        value = 0.0 if is_black else 1.0
        colors.append(Gf.Vec3f(value, value, value))

    mesh.CreatePointsAttr(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * len(colors)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    display_colors = UsdGeom.Gprim(mesh).CreateDisplayColorPrimvar(
        UsdGeom.Tokens.uniform
    )
    display_colors.Set(Vt.Vec3fArray(colors))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    return mesh
