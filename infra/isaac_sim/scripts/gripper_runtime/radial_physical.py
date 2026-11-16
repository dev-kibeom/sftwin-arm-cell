"""Prismatic radial actuation and fresh target-contact Holding, without attachment."""
import math
import time
from gripper_runtime.grasp_attachment import HoldingObservation, HoldingState
from gripper_runtime.tool_plugin import ToolIntent, ToolRuntime


class RadialPhysicalTool:
    def __init__(self, strategy, write_primary, measure_fingers, *, lower, upper,
                 speed_m_s, preload_m, tolerance_m, contact_threshold_n, dwell_s):
        self.strategy, self.write, self.measure = strategy, write_primary, measure_fingers
        values = (lower, upper, speed_m_s, preload_m, tolerance_m, contact_threshold_n, dwell_s)
        if not all(math.isfinite(v) for v in values) or not lower < upper or min(values[2:]) <= 0:
            raise ValueError('Invalid radial physical settings')
        self.lower, self.upper, self.speed = lower, upper, speed_m_s
        self.preload, self.tolerance = preload_m, tolerance_m
        self.threshold, self.dwell = contact_threshold_n, dwell_s
        self.intent = None
        self.target = max(self.lower, min(self.upper, self._positions()[0]))
        self.held_dwell = self.release_dwell = 0.
        self.sequence = 0
        self._holding = HoldingObservation(HoldingState.UNKNOWN, time.monotonic(), 0)
        self.contacts = None

    def _positions(self):
        positions = tuple(float(q) for q in self.measure())
        if len(positions) != 3 or not all(math.isfinite(q) for q in positions):
            raise RuntimeError('Three finite measured finger positions required')
        return positions

    def command(self, intent):
        self.intent = ToolIntent(intent)
        self.held_dwell = self.release_dwell = 0.
        self.target = max(self.lower, min(self.upper, self._positions()[0]))
        if self.intent is ToolIntent.STOP:
            self.write(self.target)
        self._record(HoldingState.UNKNOWN)

    def _record(self, state):
        self.sequence += 1
        self._holding = HoldingObservation(state, time.monotonic(), self.sequence)

    def update(self, dt, finger_forces_n):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Fresh positive physics dt required')
        positions = self._positions()
        if self.intent in (ToolIntent.GRASP, ToolIntent.RELEASE):
            endpoint = self.strategy.primary_position(self.intent.value)
            if self.intent is ToolIntent.GRASP:
                endpoint = max(self.lower, endpoint-self.preload)
            if not self.lower <= endpoint <= self.upper:
                raise ValueError('Radial target outside selected joint limits')
            self.target += max(-self.speed*dt, min(self.speed*dt, endpoint-self.target))
            self.write(self.target)
        valid = (finger_forces_n is not None and len(finger_forces_n) == 3
                 and all(math.isfinite(v) and v >= 0 for v in finger_forces_n))
        self.contacts = list(finger_forces_n) if valid else None
        contact = valid and all(v >= self.threshold for v in finger_forces_n)
        # End-stop obstruction or a commanded aperture alone cannot prove HELD.
        obstructed = (self.target < min(positions)-self.tolerance
                      and all(self.lower+self.tolerance < q < self.upper-self.tolerance for q in positions))
        held = self.intent is ToolIntent.GRASP and contact and obstructed
        released = (self.intent is ToolIntent.RELEASE and valid
                    and max(finger_forces_n) < self.threshold
                    and all(abs(q-self.strategy.solution['release_position_m']) <= self.tolerance for q in positions))
        self.held_dwell = self.held_dwell+dt if held else 0.
        self.release_dwell = self.release_dwell+dt if released else 0.
        state = (HoldingState.HELD if self.held_dwell >= self.dwell else
                 HoldingState.RELEASED if self.release_dwell >= self.dwell else HoldingState.UNKNOWN)
        self._record(state)

    def holding_observation(self):
        return self._holding

    def observation(self):
        return dict(positions_m=self._positions(), target_m=self.target,
                    finger_forces_n=self.contacts, holding=self._holding.state.value,
                    intent=self.intent.value if self.intent else None)


def radial_physical_runtime(profile, binding):
    """Selected Tool factory; Isaac drive and measurement services are injected."""
    from gripper_runtime.grasp_strategy import select_grasp_strategy
    settings = profile['runtime']
    strategy = select_grasp_strategy(settings, binding.requirement, geometry=binding.geometry)
    joint = settings['joint_model'][settings['primary_joint']]
    actuator = RadialPhysicalTool(strategy, binding.write_primary, binding.measure_fingers,
        lower=joint['lower'], upper=joint['upper'], speed_m_s=joint['velocity']*.25,
        **binding.physical_settings)
    return ToolRuntime(actuator, frozenset({'joint_position', 'actuator_observation'}),
        lambda intent, duration: actuator.command(intent), strategy)
