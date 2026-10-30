import pytest

from gripper_runtime.grasp_qualification import (
    ROBOTIQ_2F_85_MODEL_ID,
    GraspQualification,
    GripperCharacteristics,
)


def qualify(
    *,
    material_shape="box",
    gripper_model=ROBOTIQ_2F_85_MODEL_ID,
):
    return GraspQualification.evaluate(
        material_shape=material_shape,
        gripper_characteristics=(
            GripperCharacteristics(gripper_model) if gripper_model else None
        ),
    )


def test_box_robotiq_2f85_is_supported_without_contact_observation():
    result = qualify()

    assert result.eligible is True
    assert result.reason is None


def test_robotiq_actuator_exposes_its_gripper_characteristics():
    from gripper_runtime.robotiq_actuator import Robotiq2F85Actuator

    assert Robotiq2F85Actuator.MODEL_ID == ROBOTIQ_2F_85_MODEL_ID


@pytest.mark.parametrize(
    ("material_shape", "reason"),
    [
        (None, "shape_tag_missing"),
        ("", "shape_tag_invalid"),
        ("cylinder", "unsupported_shape"),
    ],
)
def test_missing_invalid_and_unsupported_material_shapes_fail_closed(
    material_shape, reason
):
    result = qualify(material_shape=material_shape)

    assert result.eligible is False
    assert result.reason == reason


def test_unsupported_gripper_fails_closed_for_box_material():
    result = qualify(gripper_model="other_gripper")

    assert result.eligible is False
    assert result.reason == "unsupported_gripper"
