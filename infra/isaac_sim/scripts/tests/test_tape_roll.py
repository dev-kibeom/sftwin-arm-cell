"""A dynamic roll must retain an open bore and physically meaningful inertia."""
import math
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.material import TapeRoll


def test_convex_segments_do_not_fill_bore():
    roll = TapeRoll(.06, .075, .017, .08, .8, .6)
    for vertices in roll.convex_segments():
        vertices = np.array(vertices)
        # Every convex piece lies outside a supporting bore tangent plane.
        mid = vertices[:, :2].mean(axis=0)
        normal = mid / np.linalg.norm(mid)
        assert (vertices[:, :2] @ normal).min() >= roll.inner_radius_m - 1e-12
        assert np.max(np.linalg.norm(vertices[:, :2], axis=1)) <= roll.outer_radius_m + 1e-12
        assert np.ptp(vertices[:, 2]) == pytest.approx(.017)
    ix, iy, iz = roll.inertia()
    assert ix == pytest.approx(.08 * (3*(.06**2+.075**2)+.017**2)/12)
    assert iy == ix
    assert iz == pytest.approx(.08*(.06**2+.075**2)/2)


@pytest.mark.parametrize('values', [(.07,.06,.017,.08,.8,.6),(.06,.075,0,.08,.8,.6),
    (.06,.075,.017,-1,.8,.6),(.06,.075,.017,.08,.4,.6),(.06,math.nan,.017,.08,.8,.6)])
def test_invalid_material_is_rejected(values):
    with pytest.raises(ValueError):
        TapeRoll(*values)
