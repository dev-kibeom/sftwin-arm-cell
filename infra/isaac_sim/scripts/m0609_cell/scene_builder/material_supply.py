"""Seeded, unconstrained pose sampling for the Isaac material supply adapter."""

from __future__ import annotations

import math
import random
import secrets
from dataclasses import dataclass
from typing import Any, Iterator


@dataclass(frozen=True)
class MaterialSupplyProfile:
    material_type: str
    shape: str
    dimensions_m: tuple[float, float, float]
    mass_kg: float
    receiving_x_m: tuple[float, float]
    receiving_y_m: tuple[float, float]
    spawn_height_above_shelf_m: float
    spawn_interval_s: float
    orientation_rpy_rad: tuple[tuple[float, float], ...]

    @classmethod
    def from_mapping(
        cls,
        simulation: dict[str, Any],
        vision: dict[str, Any],
        material_definitions: dict[str, Any],
    ) -> "MaterialSupplyProfile":
        supply = simulation.get("material_supply")
        if not isinstance(supply, dict):
            raise ValueError("simulation.material_supply profile is required")
        if supply.get("material_type") != "RawPart":
            raise ValueError("simulation.material_supply.material_type must be RawPart")
        target = vision.get("detector_profiles", {}).get("RawPart")
        if not isinstance(target, dict):
            raise ValueError("Vision RawPart profile is required for material supply")
        material = material_definitions.get("RawPart")
        if not isinstance(material, dict) or material.get("material_id") != "RawPart":
            raise ValueError("material_definitions.RawPart must define RawPart")
        if material.get("shape") != "box":
            raise ValueError("RawPart material shape must be box")
        if target.get("material_id") != material["material_id"]:
            raise ValueError(
                "Vision RawPart profile must reference its material definition"
            )
        dimensions = _finite_tuple(
            material.get("nominal_dimensions_m"), 3, "RawPart nominal dimensions"
        )
        if any(value <= 0 for value in dimensions):
            raise ValueError("RawPart dimensions must be positive")
        mass = _finite_positive(supply.get("mass_kg"), "mass_kg")
        support = _finite_tuple(
            target.get("support_region_dimensions_m"), 2, "support region dimensions"
        )
        fraction = _finite_positive(
            supply.get("receiving_region_fraction"), "receiving_region_fraction"
        )
        if fraction > 1.0:
            raise ValueError("receiving_region_fraction must be <= 1")
        x_half = support[0] * fraction / 2.0
        y_half = support[1] * fraction / 2.0
        x_bounds = (-x_half, x_half)
        y_bounds = (-y_half, y_half)
        height = _finite_positive(
            supply.get("spawn_height_above_shelf_m"),
            "spawn_height_above_shelf_m",
        )
        interval = _finite_positive(supply.get("spawn_interval_s"), "spawn_interval_s")
        if supply.get("position_distribution") != "uniform":
            raise ValueError("material position_distribution must be uniform")
        if supply.get("orientation_distribution") != "uniform_rpy":
            raise ValueError("material orientation_distribution must be uniform_rpy")
        angles = supply.get("orientation_rpy_rad")
        if not isinstance(angles, dict):
            raise ValueError("orientation_rpy_rad is required")
        orientation = tuple(
            _ordered_bounds(angles.get(axis), f"orientation_rpy_rad.{axis}")
            for axis in ("roll", "pitch", "yaw")
        )
        if (
            x_bounds[0] < -support[0] / 2
            or x_bounds[1] > support[0] / 2
            or y_bounds[0] < -support[1] / 2
            or y_bounds[1] > support[1] / 2
        ):
            raise ValueError("receiving region must be inside the Tray support")
        return cls(
            material_type="RawPart",
            shape=material["shape"],
            dimensions_m=dimensions,
            mass_kg=mass,
            receiving_x_m=x_bounds,
            receiving_y_m=y_bounds,
            spawn_height_above_shelf_m=height,
            spawn_interval_s=interval,
            orientation_rpy_rad=orientation,
        )


def sample_initial_poses(
    profile: MaterialSupplyProfile,
    quantity: int,
    *,
    shelf_origin_m: tuple[float, float, float],
    seed: int | None,
) -> Iterator[tuple[float, float, float, float, float, float]]:
    """Sample world XYZ and RPY; no grasp, marker, or outcome filtering occurs."""
    if type(quantity) is not int or quantity <= 0:
        raise ValueError("quantity must be a positive integer")
    if len(shelf_origin_m) != 3 or not all(_finite(value) for value in shelf_origin_m):
        raise ValueError("shelf_origin_m must contain three finite values")
    selected_seed = secrets.randbits(64) if seed is None else seed
    if type(selected_seed) is not int or not 0 <= selected_seed < 2**64:
        raise ValueError("seed must be an unsigned 64-bit integer")
    rng = random.Random(selected_seed)
    x_center, y_center, shelf_z = (float(value) for value in shelf_origin_m)
    for _ in range(quantity):
        yield (
            x_center + rng.uniform(*profile.receiving_x_m),
            y_center + rng.uniform(*profile.receiving_y_m),
            shelf_z + profile.spawn_height_above_shelf_m,
            rng.uniform(*profile.orientation_rpy_rad[0]),
            rng.uniform(*profile.orientation_rpy_rad[1]),
            rng.uniform(*profile.orientation_rpy_rad[2]),
        )


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def _finite_tuple(value: Any, length: int, name: str) -> tuple[float, ...]:
    if (
        not isinstance(value, (list, tuple))
        or len(value) != length
        or not all(_finite(item) for item in value)
    ):
        raise ValueError(f"{name} must contain {length} finite values")
    return tuple(float(item) for item in value)


def _ordered_bounds(value: Any, name: str) -> tuple[float, float]:
    bounds = _finite_tuple(value, 2, name)
    if bounds[0] >= bounds[1]:
        raise ValueError(f"{name} bounds must be increasing")
    return bounds


def _finite_positive(value: Any, name: str) -> float:
    if not _finite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)
