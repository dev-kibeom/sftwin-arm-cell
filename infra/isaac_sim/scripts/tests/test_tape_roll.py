"""A dynamic roll must retain an open bore and physically meaningful inertia."""
import math
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.material import TapeRoll


def test_ring_mesh_is_closed_outward_facing_and_preserves_bore():
    roll = TapeRoll(.06, .075, .017, .08, .8, .6)
    vertices, faces = roll.mesh()
    vertices = np.array(vertices)
    # A single connected annular surface, without internal sector faces.
    assert len(vertices) == 4 * roll.segments
    assert len(faces) == 4 * roll.segments
    assert np.ptp(vertices[:, 2]) == pytest.approx(.017)
    radii = np.linalg.norm(vertices[:, :2], axis=1)
    assert radii.max() == pytest.approx(.075)
    assert radii.min() >= .06
    edges = {}
    signed_volume = 0.
    for face in faces:
        for start, end in zip(face, face[1:] + face[:1]):
            edges[start, end] = edges.get((start, end), 0) + 1
            # No edge crosses the empty bore, including an inner chord.
            a, b = vertices[start, :2], vertices[end, :2]
            delta = b - a
            t = np.clip(-a @ delta / (delta @ delta), 0, 1) if delta @ delta else 0
            assert np.linalg.norm(a + t * delta) >= .06 - 1e-12
        a, b, c, d = vertices[list(face)]
        signed_volume += (a @ np.cross(b, c) + a @ np.cross(c, d)) / 6
    assert all(count == 1 and edges.get((end, start)) == 1
               for (start, end), count in edges.items())
    assert signed_volume == pytest.approx(math.pi * (.075**2 - .06**2) * .017, rel=.02)


def test_roll_inertia_remains_analytic_and_independent_of_collision_approximation():
    roll = TapeRoll(.06, .075, .017, .08, .8, .6)
    ix, iy, iz = roll.inertia()
    assert ix == pytest.approx(.08 * (3*(.06**2+.075**2)+.017**2)/12)
    assert iy == ix
    assert iz == pytest.approx(.08*(.06**2+.075**2)/2)


@pytest.mark.parametrize('values', [(.07,.06,.017,.08,.8,.6),(.06,.075,0,.08,.8,.6),
    (.06,.075,.017,-1,.8,.6),(.06,.075,.017,.08,.4,.6),(.06,math.nan,.017,.08,.8,.6)])
def test_invalid_material_is_rejected(values):
    with pytest.raises(ValueError):
        TapeRoll(*values)
