"""Robotiq simulation release status from measured command approach and stability.

These are Tool joint observations, not material-ground-truth evidence.
"""
import math

from gripper_runtime.position_stability import PositionStability


class RobotiqReleaseObservation:
    def __init__(self, settings, max_width_mm):
        self.settings = settings
        self.max_width_mm = max_width_mm
        self._stability = PositionStability()

    def reset(self):
        self._stability.reset()

    def update(self, observation, dt):
        try:
            pos = float(observation['actual_position_rad'])
            target = float(observation['target_position_rad'])
            lower, upper = observation['lower_limit_rad'], observation['upper_limit_rad']
            tolerance = self.settings['open_tolerance_mm']
            valid = (all(math.isfinite(v) for v in (pos, target, lower, upper, tolerance))
                     and upper > lower and self.max_width_mm > 0 and tolerance > 0
                     and observation['command_direction'] in ('opening', 'closing')
                     and math.isclose(target, lower, abs_tol=1e-9)
                     and abs(pos - target) * self.max_width_mm / (upper - lower) <= tolerance)
        except (KeyError, ValueError, TypeError, OverflowError):
            valid = False
        if not valid:
            self.reset()
            return False
        duration = self._stability.update(
            pos, dt, self.settings['position_stability_tolerance_rad'])
        return duration >= self.settings['stable_duration_s']
