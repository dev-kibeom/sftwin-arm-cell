from types import SimpleNamespace
import pytest
from projection_assertions import deproject_optical_z, require_projection_contract


def test_deprojection_requires_valid_optical_axis_depth():
    info = SimpleNamespace(k=[100.0, 0.0, 10.0, 0.0, 200.0, 20.0, 0.0, 0.0, 1.0])
    assert deproject_optical_z(110, 220, 2.0, info) == pytest.approx((2.0, 2.0, 2.0))
    with pytest.raises(ValueError):
        deproject_optical_z(0, 0, 0.0, info)


def test_missing_live_geometry_evidence_cannot_pass_as_a_contract():
    with pytest.raises(RuntimeError, match="NOT VERIFIED"):
        require_projection_contract({})
