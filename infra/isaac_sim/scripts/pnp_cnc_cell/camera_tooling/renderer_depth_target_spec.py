"""Canonical temporary renderer-depth fixtures, expressed in camera optical coordinates."""

from dataclasses import dataclass


CANONICAL_FRAME = "camera_color_optical_frame"
TARGET_DIMENSIONS_M = (0.08, 0.08, 0.004)


@dataclass(frozen=True)
class TargetDefinition:
    target_id: str
    frame_id: str
    front_optical_xyz_m: tuple[float, float, float]
    dimensions_m: tuple[float, float, float]
    rgb: tuple[float, float, float]
    material_id: str

    @property
    def optical_z_m(self):
        """The expected optical depth, derived from the canonical XYZ."""
        return self.front_optical_xyz_m[2]


@dataclass(frozen=True)
class CandidateSet:
    candidate_id: str
    targets: tuple[TargetDefinition, TargetDefinition, TargetDefinition]
    source: str


_TARGET_STYLE = (
    ("z080", (0.95, 0.10, 0.10), "renderer_depth_red"),
    ("z110", (0.10, 0.95, 0.10), "renderer_depth_green"),
    ("z140", (0.10, 0.20, 0.95), "renderer_depth_blue"),
)


def _target(target_id, xyz, rgb, material_id):
    return TargetDefinition(
        target_id=target_id,
        frame_id=CANONICAL_FRAME,
        front_optical_xyz_m=tuple(xyz),
        dimensions_m=TARGET_DIMENSIONS_M,
        rgb=rgb,
        material_id=material_id,
    )


CANONICAL_TARGETS = (
    _target(
        "z080",
        (-0.2739429677526156, -0.12114609330892565, 0.80),
        *_TARGET_STYLE[0][1:],
    ),
    _target(
        "z110",
        (0.3196456043049693, -0.18158271418263516, 1.10),
        *_TARGET_STYLE[1][1:],
    ),
    _target("z140", (0.5090045879905422, 0.0, 1.40), *_TARGET_STYLE[2][1:]),
)
TARGET_DEPTHS_M = tuple(target.optical_z_m for target in CANONICAL_TARGETS)

# Kept as a named candidate for the bounded B1 suitability tooling. It now
# points to the approved canonical fixture, rather than an alternate layout.
CANDIDATE_A = CandidateSet(
    candidate_id="A",
    targets=CANONICAL_TARGETS,
    source="architecture-approved B1 live-verified canonical fixture",
)

_FALLBACK_HORIZONTAL_FRACTIONS = (0.25, 0.50, 0.75)
_FALLBACK_LAYOUTS = (
    ("B", (0.50, 0.50, 0.50)),
    ("C", (0.40, 0.50, 0.60)),
)


def fallback_candidates(camera_info):
    """Return a finite deterministic set of layouts positioned from live FOV.

    Target centers are placed at fixed image fractions; the current principal
    point and focal lengths convert those centers to optical-frame X/Y at the
    three canonical depths. Target dimensions are included by the suitability
    evaluator's border check.
    """
    width, height = int(camera_info.width), int(camera_info.height)
    k = camera_info.k
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if width <= 0 or height <= 0 or fx <= 0 or fy <= 0:
        raise ValueError("CameraInfo dimensions and focal lengths must be positive")

    candidates = []
    for candidate_id, vertical_fractions in _FALLBACK_LAYOUTS:
        targets = []
        for (target_id, rgb, material_id), z, x_fraction, y_fraction in zip(
            _TARGET_STYLE,
            TARGET_DEPTHS_M,
            _FALLBACK_HORIZONTAL_FRACTIONS,
            vertical_fractions,
        ):
            pixel_x = x_fraction * width
            pixel_y = y_fraction * height
            optical_xyz = (
                (pixel_x - cx) * z / fx,
                (pixel_y - cy) * z / fy,
                z,
            )
            targets.append(_target(target_id, optical_xyz, rgb, material_id))
        candidates.append(
            CandidateSet(
                candidate_id=candidate_id,
                targets=tuple(targets),
                source="deterministic current-FOV layout; fixed image fractions",
            )
        )
    return tuple(candidates)
