"""Finite profile-sized RawPart layouts for the Simulation UI Hub."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class _Placement:
    x: float
    y: float
    face: str = "a×b"
    yaw: float = 0.0
    roll: float = 0.0
    layer: int = 0
    support: int | None = None


# The quaternion maps the selected local dimension axis to world +Z.
_FACE_UP = {
    "a×b": (2, (0.0, 0.0, 0.0, 1.0)),
    "a×c": (1, (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))),
    "b×c": (0, (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5))),
}
_ONE_LAYER = {
    "single",
    "side_by_side",
    "angled_adjacent",
    "l_shape",
    "u_shape",
    "u_shape_five",
    "mixed_faces",
}


def scenarios() -> tuple[tuple[str, str, str], ...]:
    return (
        ("single", "Single", "One RawPart, a×b face up."),
        ("side_by_side", "Side by Side", "Two matching faces in contact."),
        ("angled_adjacent", "Angled Adjacent", "Adjacent parts with different yaw."),
        ("l_shape", "L Shape", "Three parts forming an L outline."),
        (
            "u_shape",
            "U Shape · 3 Parts",
            "Three rotated parts forming an open U outline.",
        ),
        (
            "u_shape_five",
            "U Shape · 5 Parts",
            "Five same-face parts forming a U outline with an open center.",
        ),
        (
            "mixed_faces",
            "Mixed Faces",
            "Adjacent parts with different face-up heights.",
        ),
        ("two_single", "2 Layer · Single", "One part supported by one lower part."),
        (
            "two_adjacent",
            "2 Layer · Adjacent",
            "An adjacent lower pattern with an upper pair.",
        ),
        (
            "two_mixed",
            "2 Layer · Mixed Faces",
            "Two layers combining different face-up dimensions.",
        ),
        (
            "two_tilted",
            "2 Layer · Tilted",
            "An upper part tilted above a supported lower pattern.",
        ),
        (
            "three_single",
            "3 Layer · Single",
            "Single part per layer, up to three levels.",
        ),
        (
            "three_patterns",
            "3 Layer · Patterns",
            "L lower pattern, pair middle, upper part on a different face.",
        ),
        (
            "three_tilted",
            "3 Layer · Tilted",
            "Two regular levels with a tilted upper part.",
        ),
    )


def display_name(scenario_id: str, name: str) -> str:
    return f"1 Layer · {name}" if scenario_id in _ONE_LAYER else name


def _placements(scenario_id: str, a: float, b: float, c: float) -> list[_Placement]:
    one = _Placement
    pair = [one(-a / 2, 0), one(a / 2, 0)]
    l_pattern = pair + [one(-a / 2, b)]
    # Two long arms rotated across the narrow dimension, plus a bridge.
    # Their edges meet exactly for any rectangular material profile.
    u_pattern = [
        one(-(a + b) / 2, 0, yaw=math.pi / 2),
        one((a + b) / 2, 0, yaw=math.pi / 2),
        one(0, (a + b) / 2),
    ]
    u_pattern_five = [
        one(-a, b / 2),
        one(0, b / 2),
        one(a, b / 2),
        one(-a, -b / 2),
        one(a, -b / 2),
    ]
    angle = math.pi / 8
    # Place the facing support vertices at the same point. The rotated
    # rectangles lie on opposite sides of that vertical support plane.
    angled_offset = (a * math.cos(angle) + b * math.sin(angle)) / 2
    angled_y = (a * math.sin(angle) - b * math.cos(angle)) / 2
    angled = [
        one(-angled_offset, angled_y, yaw=-angle),
        one(angled_offset, angled_y, yaw=angle),
    ]

    if scenario_id == "single":
        return [one(0, 0)]
    if scenario_id == "side_by_side":
        return pair
    if scenario_id == "angled_adjacent":
        return angled
    if scenario_id == "l_shape":
        return l_pattern
    if scenario_id == "u_shape":
        return u_pattern
    if scenario_id == "u_shape_five":
        return u_pattern_five
    if scenario_id == "mixed_faces":
        return [one(-a / 2, 0), one(a / 2, 0, face="a×c")]
    if scenario_id == "two_single":
        return [one(0, 0, layer=0), one(0, 0, layer=1, support=0)]
    if scenario_id == "two_adjacent":
        return pair + [
            one(-a / 2, 0, layer=1, support=0),
            one(a / 2, 0, layer=1, support=1),
        ]
    if scenario_id == "two_mixed":
        support = 0 if c >= b else 1
        taller_face_x = -a / 2 if support == 0 else a / 2
        return [
            one(-a / 2, 0, face="a×b"),
            one(a / 2, 0, face="a×c"),
            one(taller_face_x, 0, face="b×c", layer=1, support=support),
        ]
    if scenario_id == "two_tilted":
        lower = [
            one(-a / 2, 0),
            one(a / 2, -b / 2),
            one(a / 2, b / 2),
        ]
        return lower + [
            one(-a / 2, 0, layer=1, support=0),
            one(a / 2, 0, roll=math.radians(18), layer=1, support=1),
        ]
    if scenario_id == "three_single":
        return [
            one(0, 0, layer=level, support=level - 1 if level else None)
            for level in range(3)
        ]
    if scenario_id == "three_patterns":
        return l_pattern + [
            one(-a / 2, 0, layer=1, support=0),
            one(a / 2, 0, layer=1, support=1),
            one(-a / 2, 0, face="b×c", layer=2, support=3),
        ]
    if scenario_id == "three_tilted":
        support_pattern = [
            one(-a / 2, 0),
            one(a / 2, -b / 2),
            one(a / 2, b / 2),
        ]
        middle = [
            one(item.x, item.y, layer=1, support=index)
            for index, item in enumerate(support_pattern)
        ]
        upper = [
            one(-a / 2, 0, layer=2, support=3),
            one(a / 2, 0, roll=math.radians(18), layer=2, support=4),
        ]
        return support_pattern + middle + upper
    raise ValueError(f"unknown Cuboid Demonstration: {scenario_id}")


def build_layout(
    scenario_id: str,
    dimensions: tuple[float, float, float],
    origin: tuple[float, float, float],
) -> list[dict[str, Any]]:
    """Compose single-layer patterns and derive each layer's contact height."""
    if len(dimensions) != 3 or len(origin) != 3:
        raise ValueError("RawPart dimensions and origin must be XYZ triples")
    a, b, c = dimensions
    if any(not math.isfinite(value) or value <= 0 for value in dimensions):
        raise ValueError("RawPart dimensions must be finite and positive")
    if any(not math.isfinite(value) for value in origin):
        raise ValueError("Tray origin must be finite")

    placements = _placements(scenario_id, a, b, c)
    output = []
    world_centers: list[float] = []
    for index, item in enumerate(placements):
        face_quaternion = _FACE_UP[item.face][1]
        qx, qy, qz, qw = face_quaternion
        sin_roll, cos_roll = math.sin(item.roll / 2), math.cos(item.roll / 2)
        rolled = (
            cos_roll * qx + sin_roll * qw,
            cos_roll * qy - sin_roll * qz,
            cos_roll * qz + sin_roll * qy,
            cos_roll * qw - sin_roll * qx,
        )
        sin_yaw, cos_yaw = math.sin(item.yaw / 2), math.cos(item.yaw / 2)
        # Compose world yaw * world roll * face-up quaternion.
        rx, ry, rz, rw = rolled
        orientation = (
            cos_yaw * rx - sin_yaw * ry,
            cos_yaw * ry + sin_yaw * rx,
            cos_yaw * rz + sin_yaw * rw,
            cos_yaw * rw - sin_yaw * rz,
        )
        ox, oy, oz, ow = orientation
        world_up_projection = (
            2 * (ox * oz - ow * oy),
            2 * (oy * oz + ow * ox),
            1 - 2 * (ox * ox + oy * oy),
        )
        projected_half_height = 0.5 * sum(
            abs(up_component) * dimension
            for up_component, dimension in zip(world_up_projection, dimensions)
        )
        if item.support is None:
            base_z = 0.0
        else:
            if (
                item.support >= index
                or placements[item.support].layer != item.layer - 1
            ):
                raise ValueError(
                    "each stacked RawPart must reference the preceding layer"
                )
            # The supporting RawPart's upper extent uses its own real pose.
            support_orientation = output[item.support]["orientation_xyzw"]
            sx, sy, sz, sw = support_orientation
            support_up = (
                2 * (sx * sz + sw * sy),
                2 * (sy * sz - sw * sx),
                1 - 2 * (sx * sx + sy * sy),
            )
            support_half_height = 0.5 * sum(
                abs(component) * dimension
                for component, dimension in zip(support_up, dimensions)
            )
            base_z = world_centers[item.support] + support_half_height
        center_z = base_z + projected_half_height
        world_centers.append(center_z)
        output.append(
            {
                "position_xyz": [
                    origin[0] + item.x,
                    origin[1] + item.y,
                    origin[2] + center_z,
                ],
                "orientation_xyzw": list(orientation),
                "dimensions_m": list(dimensions),
            }
        )
    return output
