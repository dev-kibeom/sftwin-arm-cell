"""Pure grasp and capture acceptance policies for the M0609 runtime."""

from dataclasses import dataclass
from enum import Enum
import math


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
    """Explicit validation-profile configuration for capture-local snapping."""

    capture_volume_dimensions_m: tuple
    position_tolerance_m: float
    orientation_tolerance_deg: float
    expected_contact_width_mm: float
    contact_width_tolerance_mm: float

    def validate(self):
        dimensions = self.capture_volume_dimensions_m
        if dimensions is None or len(dimensions) != 3:
            raise ValueError("capture_volume_dimensions_m must have three values")
        values = (
            *dimensions,
            self.position_tolerance_m,
            self.orientation_tolerance_deg,
            self.expected_contact_width_mm,
            self.contact_width_tolerance_mm,
        )
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError("capture configuration must be finite")
        if any(float(value) <= 0.0 for value in values):
            raise ValueError("capture configuration must be positive")
        if self.orientation_tolerance_deg > 180.0:
            raise ValueError("orientation_tolerance_deg must be <= 180")
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


@dataclass(frozen=True)
class CaptureResult:
    decision: CaptureDecision
    candidate: CaptureCandidate = None


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
            return CaptureResult(CaptureDecision.INVALID_CONFIG)

        if not runtime_available:
            return CaptureResult(CaptureDecision.UNKNOWN_RUNTIME)
        if not close_active or owned_joint_path is not None:
            return CaptureResult(CaptureDecision.NO_CAPTURE)

        accepted = []
        for candidate in candidates:
            if not candidate.eligible:
                continue
            if not CapturePolicy._candidate_is_finite(candidate):
                return CaptureResult(CaptureDecision.UNKNOWN_RUNTIME)
            if not CapturePolicy._inside_volume(
                candidate.tcp_local_position_m,
                config.capture_volume_dimensions_m,
            ):
                continue
            if candidate.position_error_m > config.position_tolerance_m:
                continue
            if candidate.orientation_error_deg > config.orientation_tolerance_deg:
                continue
            width_error = abs(
                candidate.measured_opening_mm - config.expected_contact_width_mm
            )
            if width_error > config.contact_width_tolerance_mm:
                continue
            accepted.append(candidate)

        if len(accepted) == 0:
            return CaptureResult(CaptureDecision.NO_CAPTURE)
        if len(accepted) > 1:
            return CaptureResult(CaptureDecision.AMBIGUOUS)
        return CaptureResult(CaptureDecision.CAPTURE_ELIGIBLE, accepted[0])

    @staticmethod
    def _candidate_is_finite(candidate):
        if len(candidate.tcp_local_position_m) != 3:
            return False
        values = (
            *candidate.tcp_local_position_m,
            candidate.position_error_m,
            candidate.orientation_error_deg,
            candidate.measured_opening_mm,
        )
        return all(math.isfinite(float(value)) for value in values)

    @staticmethod
    def _inside_volume(position, dimensions):
        return all(
            abs(float(value)) <= float(size) / 2.0
            for value, size in zip(position, dimensions)
        )
