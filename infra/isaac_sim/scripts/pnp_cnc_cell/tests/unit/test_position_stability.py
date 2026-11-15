import pytest
from gripper_runtime.position_stability import PositionStability


def test_cumulative_drift_resets_even_when_each_increment_is_small():
    observed=PositionStability()
    for i in range(10):
        assert observed.update(i*0.006,0.05,0.01)<0.15


@pytest.mark.parametrize('dt',[0.,-1.,float('nan'),float('inf')])
def test_invalid_time_discards_old_evidence(dt):
    observed=PositionStability()
    observed.update(0.4,0.1,0.01)
    assert observed.update(0.4,dt,0.01)==0.
    assert observed.update(0.4,0.05,0.01)==0.
    assert observed.update(0.4,0.05,0.01)==pytest.approx(0.05)


def test_position_range_includes_both_oscillation_extrema():
    observed=PositionStability()
    observed.update(0.,0.05,0.01)
    observed.update(-0.009,0.05,0.01)
    assert observed.update(0.009,0.05,0.01)==0.
