"""Tape-roll dimensions in metres, mass in kg, inertia in kg m²."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TapeRoll:
    inner_radius_m: float
    outer_radius_m: float
    width_m: float
    mass_kg: float
    static_friction: float
    dynamic_friction: float
    diagonal_inertia: tuple | None = None
    segments: int = 64

    def __post_init__(self):
        values = (self.inner_radius_m, self.outer_radius_m, self.width_m, self.mass_kg,
                  self.static_friction, self.dynamic_friction)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Roll dimensions, mass and friction must be finite and positive')
        if self.inner_radius_m >= self.outer_radius_m or self.dynamic_friction > self.static_friction:
            raise ValueError('Invalid roll radii or friction ordering')
        if not isinstance(self.segments, int) or self.segments < 12:
            raise ValueError('Roll mesh requires at least twelve radial segments')
        if self.inner_radius_m / math.cos(math.pi/self.segments) >= self.outer_radius_m:
            raise ValueError('Sector resolution is too low for this annulus')
        if self.diagonal_inertia is not None:
            inertia = self.diagonal_inertia
            if (len(inertia) != 3 or not all(math.isfinite(v) and v > 0 for v in inertia)
                    or any(2*v > sum(inertia) for v in inertia)):
                raise ValueError('Invalid principal inertia')

    def inertia(self):
        if self.diagonal_inertia is not None:
            return tuple(self.diagonal_inertia)
        radial = self.inner_radius_m**2 + self.outer_radius_m**2
        transverse = self.mass_kg * (3*radial + self.width_m**2) / 12
        return transverse, transverse, self.mass_kg * radial / 2

    def mesh(self):
        """One watertight annular mesh for visual and automatic collision cooking."""
        # Keep the inner chords outside the physical bore, as with the former
        # sector construction. segments controls surface resolution, not a
        # manually prescribed number of collision shapes.
        inner = self.inner_radius_m / math.cos(math.pi / self.segments)
        vertices = [(radius * math.cos(2 * math.pi * i / self.segments),
                     radius * math.sin(2 * math.pi * i / self.segments), z)
                    for z in (-self.width_m / 2, self.width_m / 2)
                    for radius in (inner, self.outer_radius_m)
                    for i in range(self.segments)]
        n = self.segments
        faces = []
        for i in range(n):
            j = (i + 1) % n
            faces.extend(((n+i, n+j, 3*n+j, 3*n+i),       # outer wall
                          (j, i, 2*n+i, 2*n+j),           # bore wall
                          (2*n+i, 3*n+i, 3*n+j, 2*n+j),   # top annulus
                          (j, n+j, n+i, i)))              # bottom annulus
        return vertices, faces
