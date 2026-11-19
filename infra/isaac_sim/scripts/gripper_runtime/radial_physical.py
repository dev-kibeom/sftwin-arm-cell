"""Radial actuation and position-based object detection, without scene queries."""
import math
import time
from gripper_runtime.grasp_attachment import HoldingObservation, HoldingState
from gripper_runtime.tool_plugin import ToolIntent, ToolRuntime
from gripper_runtime.robotiq_actuator import GripperState
from gripper_runtime.position_stability import PositionStability


class RadialPhysicalTool:
    def __init__(self, strategy, write_primary, measure_primary, *, lower, upper,
                 speed_m_s, preload_m, tolerance_m, dwell_s):
        self.strategy, self.write, self.measure = strategy, write_primary, measure_primary
        values = (lower, upper, speed_m_s, preload_m, tolerance_m, dwell_s)
        if not all(math.isfinite(v) for v in values) or not lower < upper or min(values[2:]) <= 0:
            raise ValueError('Invalid radial physical settings')
        self.lower, self.upper, self.speed = lower, upper, speed_m_s
        self.preload, self.tolerance = preload_m, tolerance_m
        self.dwell = dwell_s
        self.intent = None
        self.target = max(self.lower, min(self.upper, self._position()))
        self.held_dwell = self.release_dwell = 0.
        self.sequence = 0
        self._holding = HoldingObservation(HoldingState.UNKNOWN, time.monotonic(), 0)
        self.state = GripperState.IDLE
        self.stability = PositionStability()
        self.suspended = False

    def _position(self):
        position = float(self.measure())
        if not math.isfinite(position):
            raise RuntimeError('Finite primary encoder position required')
        return position

    def command(self, intent):
        if self.suspended:
            raise ValueError('Stopped radial Tool requires a new Play-time binding')
        self.intent = ToolIntent(intent)
        self.state = (GripperState.CLOSING if self.intent is ToolIntent.GRASP else
                      GripperState.OPENING if self.intent is ToolIntent.RELEASE else GripperState.IDLE)
        self.held_dwell = self.release_dwell = 0.
        self.stability.reset()
        self.target = max(self.lower, min(self.upper, self._position()))
        if self.intent is ToolIntent.STOP:
            self.write(self.target)
        self._record(HoldingState.UNKNOWN)

    def _record(self, state):
        self.sequence += 1
        self._holding = HoldingObservation(state, time.monotonic(), self.sequence)

    def reset_stopped_state(self):
        # No measured-state reads or solver writes after Timeline Stop.
        self.suspended = True
        self.intent = None
        self.state = GripperState.IDLE
        self.target = self.upper
        self.held_dwell = self.release_dwell = 0.
        self._record(HoldingState.UNKNOWN)

    def cancel_motion(self):
        self.command(ToolIntent.STOP)

    def update(self, dt):
        if self.suspended:
            self._record(HoldingState.UNKNOWN)
            return
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Fresh positive physics dt required')
        position = self._position()
        if self.intent in (ToolIntent.GRASP, ToolIntent.RELEASE):
            endpoint = self.strategy.primary_position(self.intent.value)
            if self.intent is ToolIntent.GRASP:
                endpoint = max(self.lower, endpoint-self.preload)
            if not self.lower <= endpoint <= self.upper:
                raise ValueError('Radial target outside selected joint limits')
            self.target += max(-self.speed*dt, min(self.speed*dt, endpoint-self.target))
            self.write(self.target)
        # The primary position is the device encoder equivalent. Neither the
        # contact body identity nor per-contact impulse is a device observation.
        obstructed = (self.target < position-self.tolerance and
                      self.lower+self.tolerance < position < self.upper-self.tolerance)
        if self.intent is ToolIntent.GRASP and obstructed:
            self.held_dwell = self.stability.update(position, dt, self.tolerance)
        else:
            self.stability.reset()
            self.held_dwell = 0.
        released = (self.intent is ToolIntent.RELEASE and
                    abs(position-self.strategy.solution['release_position_m']) <= self.tolerance)
        self.release_dwell = self.release_dwell+dt if released else 0.
        state = (HoldingState.HELD if self.held_dwell >= self.dwell else
                 HoldingState.RELEASED if self.release_dwell >= self.dwell else HoldingState.UNKNOWN)
        if state is HoldingState.HELD:
            self.state = GripperState.HOLDING
        elif state is HoldingState.RELEASED:
            self.state = GripperState.IDLE
        self._record(state)

    def holding_observation(self):
        return self._holding

    def observation(self):
        return dict(primary_position_m=None if self.suspended else self._position(), target_m=self.target,
                    holding=self._holding.state.value,
                    intent=self.intent.value if self.intent else None)


def radial_physical_runtime(profile, binding, *, configuration=None):
    """Build the selected production Tool; injected services remain usable in tests."""
    if configuration is not None:
        from gripper_runtime.radial_binding import IsaacRadialBinding
        binding = IsaacRadialBinding(configuration, binding)
    from gripper_runtime.radial_manager import RadialCapture, RadialGraspManager
    settings = profile['runtime']
    capture = RadialCapture(settings, binding.geometry)
    joint = settings['joint_model'][settings['primary_joint']]
    # Initial contraction is legal without a selected material; expansion needs
    # the accepted bore reference and never invents an aperture/width API.
    from types import SimpleNamespace
    initial = SimpleNamespace(solution={'release_position_m': joint['upper']},
        primary_position=lambda intent: joint['upper'] if intent == 'RELEASE' else
            (_ for _ in ()).throw(ValueError('GRASP requires a selected bore')))
    actuator = RadialPhysicalTool(initial, binding.write_primary, binding.measure_primary,
        lower=joint['lower'], upper=joint['upper'],
        speed_m_s=joint['velocity']*settings.get('speed_fraction', .25),
        **binding.physical_settings)
    tool = ToolRuntime(actuator, frozenset({'joint_position', 'actuator_observation'}),
        lambda intent, duration: actuator.command(intent), capture)
    tool.grasp_factory = lambda **services: RadialGraspManager(binding=binding, **services)
    return tool
