"""Pure grasp and capture acceptance policies for the M0609 runtime."""

import math
from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class GraspConfig:
    """Legacy pose/contact thresholds retained for compatibility."""

    position_tolerance_m: float = 0.015
    orientation_tolerance_deg: float = 12.0
    contact_width_tolerance_mm: float = 2.0

    def validate(self):
        if self.position_tolerance_m < 0.0:
            raise ValueError("position_tolerance_m must be >= 0")
        if not (0.0 <= self.orientation_tolerance_deg <= 180.0):
            raise ValueError("orientation_tolerance_deg must be in [0, 180]")
        if self.contact_width_tolerance_mm < 0.0:
            raise ValueError("contact_width_tolerance_mm must be >= 0")
        return self


class CaptureDecision(Enum):
    NO_CAPTURE = "no_capture"
    CAPTURE_ELIGIBLE = "capture_eligible"
    AMBIGUOUS = "ambiguous"
    INVALID_CONFIG = "invalid_config"
    UNKNOWN_RUNTIME = "unknown_runtime"


@dataclass(frozen=True)
class CaptureConfig:
    """Capture volume used to qualify registered objects for snap attachment."""

    capture_volume_dimensions_m: tuple

    def validate(self):
        dimensions = self.capture_volume_dimensions_m
        if dimensions is None or len(dimensions) != 3:
            raise ValueError("capture_volume_dimensions_m must have three values")
        values = tuple(dimensions)
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError("capture configuration must be finite")
        if any(float(value) <= 0.0 for value in values):
            raise ValueError("capture volume dimensions must be positive")
        return self


@dataclass(frozen=True)
class CaptureCandidate:
    """Stage-derived candidate data expressed at the policy boundary."""

    prim_path: str
    eligible: bool
    tcp_local_position_m: tuple
    position_error_m: float
    orientation_error_deg: float
    measured_opening_mm: float
    expected_orientation_error_deg: float = None
    rawpart_orientation_error_deg: float = None
    rawpart_rejection_reason: str = None
    tcp_stage_world_pose: dict = None
    stage_world_pose: dict = None
    relative_pose: dict = None
    capture_reference: dict = None
    shape: str = None


@dataclass(frozen=True)
class CaptureResult:
    decision: CaptureDecision
    candidate: CaptureCandidate = None
    assessments: tuple = ()


@dataclass(frozen=True)
class CaptureAssessment:
    prim_path: str
    accepted: bool
    rejection_reason: str = None
    capture_volume_inside: bool = None
    position_error_m: float = None
    position_tolerance_m: float = None
    orientation_error_deg: float = None
    orientation_tolerance_deg: float = None
    contact_width_error_mm: float = None
    contact_width_tolerance_mm: float = None
    position_check_evaluated: bool = False
    orientation_check_evaluated: bool = False
    contact_width_check_evaluated: bool = False


class CapturePolicy:
    """Pure, Isaac-independent capture predicate and candidate selector."""

    @staticmethod
    def evaluate(
        *,
        close_active,
        candidates,
        config,
        runtime_available,
        owned_joint_path=None,
    ):
        try:
            if config is None:
                raise ValueError("capture configuration is required")
            config.validate()
        except (AttributeError, TypeError, ValueError):
            return CaptureResult(
                CaptureDecision.INVALID_CONFIG,
                assessments=(CaptureAssessment("", False, "invalid_config"),),
            )

        if not runtime_available:
            return CaptureResult(
                CaptureDecision.UNKNOWN_RUNTIME,
                assessments=(CaptureAssessment("", False, "unknown_runtime"),),
            )
        if not close_active or owned_joint_path is not None:
            return CaptureResult(CaptureDecision.NO_CAPTURE)

        accepted = []
        assessments = []
        if not candidates:
            assessments.append(CaptureAssessment("", False, "no_capture_candidate"))
        for candidate in candidates:
            if not candidate.eligible:
                assessments.append(
                    CaptureAssessment(
                        candidate.prim_path, False, "candidate_ineligible"
                    )
                )
                continue
            if not CapturePolicy._position_is_finite(candidate):
                assessments.append(
                    CaptureAssessment(
                        candidate.prim_path, False, "candidate_pose_non_finite"
                    )
                )
                return CaptureResult(
                    CaptureDecision.UNKNOWN_RUNTIME, assessments=tuple(assessments)
                )
            inside_volume = CapturePolicy._inside_volume(
                candidate.tcp_local_position_m,
                config.capture_volume_dimensions_m,
            )
            if not inside_volume:
                assessments.append(
                    CaptureAssessment(
                        candidate.prim_path,
                        False,
                        "outside_capture_volume",
                        capture_volume_inside=False,
                        position_error_m=candidate.position_error_m,
                        orientation_error_deg=candidate.orientation_error_deg,
                    )
                )
                continue
            assessments.append(
                CaptureAssessment(
                    candidate.prim_path,
                    True,
                    None,
                    capture_volume_inside=inside_volume,
                    position_error_m=candidate.position_error_m,
                    orientation_error_deg=candidate.orientation_error_deg,
                )
            )
            accepted.append(candidate)

        if len(accepted) == 0:
            return CaptureResult(
                CaptureDecision.NO_CAPTURE, assessments=tuple(assessments)
            )
        if len(accepted) > 1:
            return CaptureResult(
                CaptureDecision.AMBIGUOUS, assessments=tuple(assessments)
            )
        return CaptureResult(
            CaptureDecision.CAPTURE_ELIGIBLE, accepted[0], tuple(assessments)
        )

    @staticmethod
    def _position_is_finite(candidate):
        try:
            if len(candidate.tcp_local_position_m) != 3:
                return False
            return all(
                math.isfinite(float(value)) for value in candidate.tcp_local_position_m
            )
        except (TypeError, ValueError, OverflowError):
            return False

    @staticmethod
    def _inside_volume(position, dimensions):
        return all(
            abs(float(value)) <= float(size) / 2.0
            for value, size in zip(position, dimensions)
        )
