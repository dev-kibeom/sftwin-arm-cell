import math
from pathlib import Path

import pytest
from scene_builder.cuboid_demonstrations import (
    _FACE_UP,
    _ONE_LAYER,
    _placements,
    build_layout,
    scenarios,
)
from scene_builder.rawpart_profile import load_rawpart_profile


@pytest.mark.parametrize("face,axis", [("a×b", 2), ("a×c", 1), ("b×c", 0)])
def test_face_up_quaternion_maps_selected_material_axis_to_world_up(face, axis):
    quaternion = _FACE_UP[face][1]
    x, y, z, w = quaternion
    vector = [0.0, 0.0, 0.0]
    vector[axis] = 1.0
    # q*v*q^-1
    qv = (x, y, z)
    cross = (
        2 * (qv[1] * vector[2] - qv[2] * vector[1]),
        2 * (qv[2] * vector[0] - qv[0] * vector[2]),
        2 * (qv[0] * vector[1] - qv[1] * vector[0]),
    )
    result = tuple(
        vector[i]
        + w * cross[i]
        + (
            qv[1] * cross[2] - qv[2] * cross[1],
            qv[2] * cross[0] - qv[0] * cross[2],
            qv[0] * cross[1] - qv[1] * cross[0],
        )[i]
        for i in range(3)
    )
    assert result == pytest.approx((0, 0, 1))


@pytest.mark.parametrize("scenario_id", [scenario[0] for scenario in scenarios()])
def test_presets_use_configured_dimensions_and_have_finite_poses(scenario_id):
    dimensions = (0.08, 0.05, 0.03)
    parts = build_layout(scenario_id, dimensions, (0.0, 0.0, 0.2))
    assert parts
    assert len(parts) <= 12
    assert all(part["dimensions_m"] == list(dimensions) for part in parts)
    assert all(
        all(math.isfinite(value) for value in part["position_xyz"]) for part in parts
    )
    assert all(
        all(math.isfinite(value) for value in part["orientation_xyzw"])
        for part in parts
    )


@pytest.mark.parametrize(
    "scenario_id,expected_count",
    [
        ("single", 1),
        ("side_by_side", 2),
        ("l_shape", 3),
        ("u_shape", 3),
        ("u_shape_five", 5),
    ],
)
def test_single_layer_pattern_sizes_are_composed_from_rawparts(
    scenario_id, expected_count
):
    parts = build_layout(scenario_id, (0.08, 0.05, 0.03), (0, 0, 0.2))
    assert len(parts) == expected_count
    assert all(part["position_xyz"][2] == pytest.approx(0.215) for part in parts)


def test_layer_heights_follow_the_named_support_and_stop_at_three_levels():
    dimensions = (0.08, 0.05, 0.03)
    two = build_layout("two_mixed", dimensions, (0, 0, 0.2))
    # The b×c upper is supported by the taller a×c RawPart (height b).
    assert two[-1]["position_xyz"][2] == pytest.approx(0.2 + 0.05 + 0.08 / 2)
    three = build_layout("three_single", dimensions, (0, 0, 0.2))
    assert [part["position_xyz"][2] for part in three] == pytest.approx(
        [0.215, 0.245, 0.275]
    )
    patterned = build_layout("three_patterns", dimensions, (0, 0, 0.2))
    # The upper b×c face has local X (dimension a) pointing upward.
    assert patterned[-1]["position_xyz"][2] == pytest.approx(
        0.2 + 0.03 + 0.03 + 0.08 / 2
    )
    assert patterned[-1]["position_xyz"][0] == pytest.approx(-dimensions[0] / 2)
    assert patterned[-1]["orientation_xyzw"][:3] == pytest.approx(
        [0, -math.sqrt(0.5), 0]
    )


@pytest.mark.parametrize("scenario_id", ["two_tilted", "three_tilted"])
def test_tilted_preset_uses_horizontal_axis_rotation_and_support_height(scenario_id):
    dimensions = (0.07, 0.07, 0.08)
    parts = build_layout(scenario_id, dimensions, (0, 0, 0.2))
    tilted = parts[-1]
    x, y, z, _w = tilted["orientation_xyzw"]
    assert abs(x) > 0.1
    assert abs(y) < 1e-9
    assert abs(z) < 1e-9
    expected_half_height = 0.5 * (
        math.sin(math.radians(18)) * dimensions[1]
        + math.cos(math.radians(18)) * dimensions[2]
    )
    placements = _placements(scenario_id, *dimensions)
    support_index = placements[-1].support
    support = parts[support_index]
    assert tilted["position_xyz"][2] == pytest.approx(
        support["position_xyz"][2] + dimensions[2] / 2 + expected_half_height
    )
    tilt_axes = _axes(tilted)
    corners = [
        tuple(
            tilted["position_xyz"][axis]
            + sum(
                signs[basis] * dimensions[basis] / 2 * tilt_axes[basis][axis]
                for basis in range(3)
            )
            for axis in range(3)
        )
        for signs in (
            (sx, sy, sz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
        )
    ]
    lowest_z = min(point[2] for point in corners)
    support_axes = _axes(support)
    lowest_points = [point for point in corners if point[2] == pytest.approx(lowest_z)]
    assert lowest_points
    for point in lowest_points:
        relative = tuple(point[i] - support["position_xyz"][i] for i in range(3))
        local = [sum(relative[i] * axis[i] for i in range(3)) for axis in support_axes]
        assert abs(local[0]) <= dimensions[0] / 2 + 1e-7
        assert abs(local[1]) <= dimensions[1] / 2 + 1e-7
        assert local[2] == pytest.approx(dimensions[2] / 2, abs=1e-7)


def _axes(part):
    x, y, z, w = part["orientation_xyzw"]
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)),
        (2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)),
        (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)),
    )


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _obb_overlap_on_axis(first, second, axis):
    length = math.sqrt(sum(value * value for value in axis))
    if length < 1e-9:
        return math.inf
    axis = tuple(value / length for value in axis)
    first_axes, second_axes = _axes(first), _axes(second)
    first_radius = sum(
        half * abs(sum(axis[i] * basis[i] for i in range(3)))
        for half, basis in zip(
            (value / 2 for value in first["dimensions_m"]), first_axes
        )
    )
    second_radius = sum(
        half * abs(sum(axis[i] * basis[i] for i in range(3)))
        for half, basis in zip(
            (value / 2 for value in second["dimensions_m"]), second_axes
        )
    )
    delta = tuple(
        second["position_xyz"][i] - first["position_xyz"][i] for i in range(3)
    )
    return first_radius + second_radius - abs(sum(delta[i] * axis[i] for i in range(3)))


def _obb_axes(first, second):
    first_axes, second_axes = _axes(first), _axes(second)
    return (
        *first_axes,
        *second_axes,
        *(_cross(a, b) for a in first_axes for b in second_axes),
    )


def _projected_radius(part, axis):
    return sum(
        half * abs(sum(axis[i] * basis[i] for i in range(3)))
        for half, basis in zip(
            (value / 2 for value in part["dimensions_m"]), _axes(part)
        )
    )


def _projected_overlap(first, second, axis):
    delta = abs(
        sum(
            (second["position_xyz"][i] - first["position_xyz"][i]) * axis[i]
            for i in range(3)
        )
    )
    return _projected_radius(first, axis) + _projected_radius(second, axis) - delta


@pytest.mark.parametrize("scenario_id", [scenario[0] for scenario in scenarios()])
@pytest.mark.parametrize("dimensions", [(0.08, 0.05, 0.03), (0.06, 0.04, 0.09)])
def test_every_preset_has_no_rotated_box_penetration_and_stacks_on_its_support(
    scenario_id, dimensions
):
    origin = (0.0, 0.0, 0.2)
    parts = build_layout(scenario_id, dimensions, origin)
    placements = _placements(scenario_id, *dimensions)
    assert max(item.layer for item in placements) < 3
    for first_index, first in enumerate(parts):
        for second in parts[first_index + 1 :]:
            overlaps = [
                _obb_overlap_on_axis(first, second, axis)
                for axis in _obb_axes(first, second)
            ]
            # SAT: separated or exactly touching on at least one true OBB axis.
            assert min(overlaps) <= 1e-7, (scenario_id, first, second, min(overlaps))
    for index, placement in enumerate(placements):
        if placement.support is None:
            assert placement.layer == 0
            continue
        support = parts[placement.support]
        lower = support["position_xyz"][2] + _projected_radius(support, (0, 0, 1))
        upper = parts[index]["position_xyz"][2] - sum(
            abs(component) * half
            for component, half in zip(
                _axes(parts[index])[2], (d / 2 for d in dimensions)
            )
        )
        assert upper == pytest.approx(lower, abs=1e-7)
        assert _projected_overlap(support, parts[index], (1, 0, 0)) > 1e-7
        assert _projected_overlap(support, parts[index], (0, 1, 0)) > 1e-7
    project_root = Path(__file__).resolve().parents[6]
    support_dimensions = load_rawpart_profile(project_root)[
        "support_region_dimensions_m"
    ]
    for part in parts:
        assert _projected_radius(part, (1, 0, 0)) <= support_dimensions[0] / 2
        assert _projected_radius(part, (0, 1, 0)) <= support_dimensions[1] / 2


def test_u_shape_has_open_end_and_three_pairwise_touching_members():
    dimensions = (0.08, 0.05, 0.03)
    parts = build_layout("u_shape", dimensions, (0, 0, 0.2))
    placements = _placements("u_shape", *dimensions)
    assert len(parts) == 3
    assert {item[0]: item[1] for item in scenarios()}["u_shape"] == (
        "U Shape · 3 Parts"
    )
    assert [item.yaw for item in placements] == pytest.approx(
        [math.pi / 2, math.pi / 2, 0]
    )
    # The U's open side is at negative Y; the bridge closes the positive-Y end.
    assert parts[0]["position_xyz"][1] == pytest.approx(parts[1]["position_xyz"][1])
    assert parts[2]["position_xyz"][1] > parts[0]["position_xyz"][1]
    assert all(
        min(
            _obb_overlap_on_axis(parts[i], parts[j], axis)
            for axis in _obb_axes(parts[i], parts[j])
        )
        == pytest.approx(0, abs=1e-7)
        for i, j in ((0, 2), (1, 2))
    )


@pytest.mark.parametrize("dimensions", [(0.08, 0.05, 0.03), (0.06, 0.04, 0.09)])
def test_u_shape_five_uses_five_same_face_parts_with_a_connected_open_center(
    dimensions,
):
    origin = (0.0, 0.0, 0.2)
    a, b, c = dimensions
    parts = build_layout("u_shape_five", dimensions, origin)
    placements = _placements("u_shape_five", *dimensions)
    scenarios_by_id = {
        scenario_id: (name, description)
        for scenario_id, name, description in scenarios()
    }

    assert len(parts) == 5
    assert scenarios_by_id["u_shape_five"][0] == "U Shape · 5 Parts"
    assert "u_shape_five" in _ONE_LAYER
    assert all(item.layer == 0 for item in placements)
    assert all(item.face == "a×b" and item.yaw == 0 for item in placements)
    assert all(
        part["position_xyz"][2] == pytest.approx(origin[2] + c / 2) for part in parts
    )
    assert all(
        part["orientation_xyzw"] == pytest.approx((0, 0, 0, 1)) for part in parts
    )

    # Each intended seam has positive edge length in one horizontal axis and
    # vertical overlap, while the other horizontal axis meets at one boundary.
    for first_index, second_index, touching_axis in (
        (0, 1, (1, 0, 0)),
        (1, 2, (1, 0, 0)),
        (0, 3, (0, 1, 0)),
        (2, 4, (0, 1, 0)),
    ):
        first, second = parts[first_index], parts[second_index]
        overlaps = [
            _obb_overlap_on_axis(first, second, axis)
            for axis in _obb_axes(first, second)
        ]
        assert min(overlaps) == pytest.approx(0, abs=1e-7)
        assert all(overlap >= -1e-7 for overlap in overlaps)
        assert _projected_overlap(first, second, touching_axis) == pytest.approx(
            0, abs=1e-7
        )
        edge_axis = (0, 1, 0) if touching_axis == (1, 0, 0) else (1, 0, 0)
        assert _projected_overlap(first, second, edge_axis) == pytest.approx(
            b if edge_axis == (0, 1, 0) else a, abs=1e-7
        )
        assert _projected_overlap(first, second, (0, 0, 1)) == pytest.approx(c)

    # The missing lower middle cell stays empty and is bounded by the U arms
    # and crossbar. SAT against that cell only allows its boundary contacts.
    empty_cell = {
        "position_xyz": [origin[0], origin[1] - b / 2, origin[2] + c / 2],
        "orientation_xyzw": [0, 0, 0, 1],
        "dimensions_m": list(dimensions),
    }
    assert all(
        min(
            _obb_overlap_on_axis(empty_cell, part, axis)
            for axis in _obb_axes(empty_cell, part)
        )
        == pytest.approx(0, abs=1e-7)
        for part in parts
    )


def test_angled_adjacent_parts_touch_at_their_actual_rotated_support_vertices():
    parts = build_layout("angled_adjacent", (0.08, 0.05, 0.03), (0, 0, 0.2))
    overlaps = [
        _obb_overlap_on_axis(parts[0], parts[1], axis) for axis in _obb_axes(*parts)
    ]
    assert min(overlaps) == pytest.approx(0, abs=1e-7)
    assert all(overlap >= -1e-7 for overlap in overlaps)


@pytest.mark.parametrize(
    "scenario_id,contacts",
    [
        ("single", ()),
        ("side_by_side", ((0, 1),)),
        ("angled_adjacent", ((0, 1),)),
        ("l_shape", ((0, 1), (0, 2))),
        ("u_shape", ((0, 2), (1, 2))),
        ("u_shape_five", ((0, 1), (1, 2), (0, 3), (2, 4))),
        ("mixed_faces", ((0, 1),)),
        ("two_single", ((0, 1),)),
        ("two_adjacent", ((0, 1), (0, 2), (1, 3), (2, 3))),
        ("two_mixed", ((0, 1), (1, 2))),
        ("two_tilted", ((0, 1), (1, 2), (0, 3), (1, 4))),
        ("three_single", ((0, 1), (1, 2))),
        ("three_patterns", ((0, 1), (0, 2), (0, 3), (1, 4), (3, 4), (3, 5))),
        (
            "three_tilted",
            ((0, 1), (1, 2), (0, 3), (1, 4), (2, 5), (3, 4), (3, 6), (4, 7)),
        ),
    ],
)
def test_named_patterns_and_stack_members_have_their_intended_contacts(
    scenario_id, contacts
):
    parts = build_layout(scenario_id, (0.08, 0.05, 0.03), (0, 0, 0.2))
    for first_index, second_index in contacts:
        overlaps = [
            _obb_overlap_on_axis(parts[first_index], parts[second_index], axis)
            for axis in _obb_axes(parts[first_index], parts[second_index])
        ]
        assert min(overlaps) == pytest.approx(0, abs=1e-7), scenario_id
        assert all(overlap >= -1e-7 for overlap in overlaps), scenario_id
