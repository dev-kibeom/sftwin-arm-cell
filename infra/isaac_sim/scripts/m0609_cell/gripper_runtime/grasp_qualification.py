"""PhysX-independent policy for qualifying an observed material grasp."""

from dataclasses import dataclass


ROBOTIQ_2F_85_MODEL_ID = "robotiq_2f_85"


@dataclass(frozen=True)
class GripperCharacteristics:
    """Gripper facts supplied to grasp qualification."""

    model_id: str


@dataclass(frozen=True)
class GraspQualificationResult:
    eligible: bool
    reason: str | None = None


class GraspQualification:
    """Qualify whether a material shape is supported by a gripper model."""

    @staticmethod
    def evaluate(
        *,
        material_shape: object,
        gripper_characteristics: GripperCharacteristics | None,
    ) -> GraspQualificationResult:
        if material_shape is None:
            return GraspQualificationResult(False, "shape_tag_missing")
        if not isinstance(material_shape, str) or not material_shape.strip():
            return GraspQualificationResult(False, "shape_tag_invalid")
        if material_shape != "box":
            return GraspQualificationResult(False, "unsupported_shape")

        if gripper_characteristics is None:
            return GraspQualificationResult(False, "gripper_characteristics_missing")
        if (
            not isinstance(gripper_characteristics, GripperCharacteristics)
            or not isinstance(gripper_characteristics.model_id, str)
            or not gripper_characteristics.model_id.strip()
        ):
            return GraspQualificationResult(False, "gripper_characteristics_invalid")
        if gripper_characteristics.model_id != ROBOTIQ_2F_85_MODEL_ID:
            return GraspQualificationResult(False, "unsupported_gripper")

        return GraspQualificationResult(True)
