"""Pure grasp acceptance policy for the scripted M0609 gripper runtime."""

from dataclasses import dataclass


@dataclass(frozen=True)
class GraspConfig:
    """Runtime grasp-policy configuration for SF-Twin."""

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
