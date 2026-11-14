import pytest

from gripper_runtime.grasp_policy import GraspConfig


def test_grasp_policy_preserves_default_thresholds():
    assert GraspConfig().validate() == GraspConfig(
        position_tolerance_m=0.015,
        orientation_tolerance_deg=12.0,
        contact_width_tolerance_mm=2.0,
    )


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"position_tolerance_m": -0.01}, "position_tolerance_m"),
        ({"orientation_tolerance_deg": 181.0}, "orientation_tolerance_deg"),
        ({"contact_width_tolerance_mm": -0.01}, "contact_width_tolerance_mm"),
    ],
)
def test_grasp_policy_rejects_invalid_thresholds(kwargs, message):
    with pytest.raises(ValueError, match=message):
        GraspConfig(**kwargs).validate()
