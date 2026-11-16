"""Physical release requires observed OPEN approach and continuous joint stability."""
import pytest
from gripper_runtime.physical_release import RobotiqReleaseObservation


def observation(position=0.00178, target=0., direction='opening'):
    return dict(actual_position_rad=position, target_position_rad=target,
                lower_limit_rad=0., upper_limit_rad=0.8, command_direction=direction)


def observer():
    return RobotiqReleaseObservation(dict(open_tolerance_mm=0.5,
        position_stability_tolerance_rad=0.001, stable_duration_s=0.2), 85.)


def test_near_open_stable_joint_releases_despite_instantaneous_velocity_jitter():
    release = observer()
    sample = dict(observation(), actual_velocity_rad_s=0.2)
    assert not release.update(sample, 0.1)
    assert not release.update(sample, 0.1)
    assert release.update(sample, 0.1)


def test_time_far_from_goal_or_merely_passing_goal_never_releases():
    release = observer()
    for _ in range(10):
        assert not release.update(observation(0.02), 0.1)
    assert not release.update(observation(), 0.1)
    assert not release.update(observation(0.0045), 0.1)
    assert not release.update(observation(0.0005), 0.1)
    release.reset()
    assert not release.update(observation(), 0.1)


def test_new_stop_or_other_target_cannot_reuse_open_stability():
    release = observer()
    for _ in range(3):
        result = release.update(observation(), 0.1)
    assert result
    assert not release.update(observation(direction=None), 0.1)
    assert not release.update(observation(), 0.1)
    assert not release.update(observation(target=0.4), 0.1)


def test_idempotent_open_at_lower_limit_still_requires_stability():
    release = observer()
    sample = observation(0., direction='closing')
    assert not release.update(sample, 0.1)
    assert not release.update(sample, 0.1)
    assert release.update(sample, 0.1)


@pytest.mark.parametrize('position', [float('nan'), float('inf')])
def test_invalid_observation_drops_release_evidence(position):
    release = observer()
    assert not release.update(observation(position), 0.1)
