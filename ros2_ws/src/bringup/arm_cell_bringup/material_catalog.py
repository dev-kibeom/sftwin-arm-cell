"""Validated material definitions shared through Bringup runtime exports."""
import math
import re
from pathlib import Path

import yaml


def load_material_catalog(directory):
    catalog = {}
    for path in sorted(Path(directory).glob('*.yaml')):
        material = yaml.safe_load(path.read_text())
        if not isinstance(material, dict):
            raise ValueError(f'Material must be a mapping: {path}')
        identity = material.get('material_id')
        if not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', identity):
            raise ValueError(f'Invalid material identity: {path}')
        if identity != path.stem or identity in catalog:
            raise ValueError(f'Material filename/identity mismatch: {path}')
        geometry = material.get('geometry', {})
        if geometry.get('material_id') != identity or geometry.get('shape') != 'box':
            raise ValueError(f'Material must define matching box geometry: {path}')
        _vector(geometry.get('nominal_dimensions_m'), 3, positive=True)
        offset = material.get('pick_offset', {})
        _vector(offset.get('translation_m'), 3)
        q = _vector(offset.get('quaternion_xyzw'), 4)
        if not math.isclose(sum(v*v for v in q), 1.0, abs_tol=1e-6):
            raise ValueError(f'Material offset quaternion must be normalized: {path}')
        _vector([material.get('mass_kg')], 1, positive=True)
        color = _vector(material.get('display_color_rgb'), 3)
        if any(v < 0 or v > 1 for v in color):
            raise ValueError(f'Material color must be in [0, 1]: {path}')
        catalog[identity] = material
    if not catalog:
        raise ValueError(f'Empty material catalog: {directory}')
    return catalog


def _vector(value, length, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != length or any(
        isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v)
        or (positive and v <= 0) for v in value
    ):
        raise ValueError(f'Expected {length} finite numeric values')
    return value
