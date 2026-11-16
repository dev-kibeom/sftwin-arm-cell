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
            raise ValueError('Roll requires at least twelve convex sectors')
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

    def convex_segments(self):
        # Circumscribe the bore: even the inner chord stays outside the true
        # opening. Each wedge is cooked separately; a whole-roll convex hull
        # would silently cap the hole in PhysX.
        half = math.pi / self.segments
        inner = self.inner_radius_m / math.cos(half)
        for index in range(self.segments):
            a, b = 2*index*half, 2*(index+1)*half
            face = [(radius*math.cos(angle), radius*math.sin(angle))
                    for radius, angle in ((inner,a),(self.outer_radius_m,a),
                                          (self.outer_radius_m,b),(inner,b))]
            yield [(x,y,z) for z in (-self.width_m/2,self.width_m/2) for x,y in face]
