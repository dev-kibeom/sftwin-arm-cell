"""Force-limited solver drive for the imported 2F-85 parallel linkage.

No DOF state writes: initial posture belongs to model assembly, not commands.
Dynamic Control is deprecated, but shipped and supported in Isaac Sim 5.1.
"""
import math

from gripper_runtime.robotiq_actuator import GripperState


def fraction(value, label):
    value = float(value)
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f'{label} must be finite in [0, 1]')
    return value


class Robotiq2F85PhysicsActuator:
    MAX_WIDTH_MM = 85.0

    def __init__(self, *, settings, joint_model, binding=None, runtime=None):
        self.settings = settings
        self.model = joint_model[settings['primary_joint']]
        self.lower, self.upper = self.model['lower'], self.model['upper']
        if runtime is None:
            from omni.isaac.dynamic_control import _dynamic_control as dc_module
            dc = dc_module.acquire_dynamic_control_interface()
            if not dc.is_simulating():
                raise RuntimeError('Physics actuator requires Play and one Physics frame')
            art = dc.get_articulation(binding.base_link_path)
            dof = dc.find_articulation_dof(art, settings['primary_joint'])
            if art == dc_module.INVALID_HANDLE or dof == dc_module.INVALID_HANDLE:
                raise RuntimeError('Tool primary joint is absent from the selected Articulation')
            runtime = dc, dc_module, art, dof
        self.dc, self.api, self.articulation, self.dof = runtime
        self.state = GripperState.IDLE
        self._target = None
        self._direction = None
        self._dwell = 0.0
        self.object_detection = 'unknown'
        self.speed_fraction = fraction(settings['speed_fraction'], 'speed_fraction')
        self.force_setting = fraction(settings['force_setting'], 'force_setting')
        self._configure_drive()

    def _configure_drive(self):
        props = self.dc.get_dof_properties(self.dof)
        props.drive_mode = self.api.DRIVE_FORCE
        props.stiffness = self.settings['drive_stiffness']
        props.damping = self.settings['drive_damping']
        minimum, maximum = self.settings['drive_torque_range_nm']
        # A simulation torque budget, NOT a calibrated conversion from rFR to N.
        props.max_effort = min(self.model['effort'], minimum + self.force_setting * (maximum - minimum))
        props.max_velocity = self.model['velocity'] * self.speed_fraction
        if not self.dc.set_dof_properties(self.dof, props):
            raise RuntimeError('Failed to configure Tool drive properties')

    def configure_command(self, *, target_position=None, speed_fraction=None, force_setting=None):
        # Validate atomically before changing any command/property.
        target = None if target_position is None else fraction(target_position, 'target_position')
        speed = self.speed_fraction if speed_fraction is None else fraction(speed_fraction, 'speed_fraction')
        force = self.force_setting if force_setting is None else fraction(force_setting, 'force_setting')
        if speed <= 0.0:
            raise ValueError('speed_fraction must be positive; use STOP to stop')
        self.speed_fraction, self.force_setting = speed, force
        self._configure_drive()
        if target is not None:
            self.move_to(target)

    def _read(self):
        state = self.dc.get_dof_state(self.dof, self.api.STATE_ALL)
        if not math.isfinite(state.pos) or not math.isfinite(state.vel):
            raise RuntimeError('Non-finite Tool joint observation')
        return state

    def get_position(self):
        return max(0., min(1., (self._read().pos - self.lower) / (self.upper - self.lower)))

    def get_width(self):
        # Compatibility telemetry only; not a calibrated contact distance.
        return self.MAX_WIDTH_MM * (1. - self.get_position())

    def move_to(self, q, duration=None):
        q = fraction(q, 'target_position')
        current = self.get_position()
        self._target = self.lower + q * (self.upper - self.lower)
        self._dwell = 0.0
        self.object_detection = 'in_motion'
        self._direction = GripperState.CLOSING if q >= current else GripperState.OPENING
        self.state = self._direction
        self.dc.wake_up_articulation(self.articulation)
        if not self.dc.set_dof_velocity_target(self.dof, 0.0):
            raise RuntimeError('Failed to clear Tool velocity target')
        if not self.dc.set_dof_position_target(self.dof, self._target):
            raise RuntimeError('Failed to command Tool position target')

    def update(self, dt):
        if self._target is None or self._direction is None:
            return False
        actual = self._read()
        tolerance = self.settings['position_tolerance_rad']
        stopped = abs(actual.vel) <= self.settings['stopped_velocity_rad_s']
        if abs(actual.pos - self._target) <= tolerance and stopped:
            self._dwell = 0.0
            self.object_detection = 'at_requested_position'
            self.state = GripperState.HOLDING
        else:
            self.state = self._direction
            self.object_detection = 'in_motion'
            self._dwell = self._dwell + max(0., float(dt)) if stopped else 0.0
            if self._dwell >= self.settings['blocked_dwell_s']:
                # Obstruction inferred from motion only; not an object/force sensor.
                self.object_detection = 'stopped_closing' if self.state is GripperState.CLOSING else 'stopped_opening'
                self.state = GripperState.HOLDING
                # Keep the force-limited closing target to maintain physical load.
        return self.is_moving

    def stop(self):
        actual = self._read()
        self.dc.wake_up_articulation(self.articulation)
        if not self.dc.set_dof_velocity_target(self.dof, 0.) or not self.dc.set_dof_position_target(self.dof, actual.pos):
            raise RuntimeError('Failed to hold Tool at measured position')
        self._target = actual.pos
        self._direction = None
        self.state = GripperState.HOLDING
        self.object_detection = 'unknown'
        self._dwell = 0.0

    def cancel_motion(self):
        # Lifecycle Stop may run after Physics handles have become invalid.
        self._target = None
        self._direction = None
        self._dwell = 0.0
        self.state = GripperState.IDLE
        self.object_detection = 'unknown'

    def open(self, duration=None):
        self.move_to(0.)

    def close(self, duration=None):
        self.move_to(1.)

    def set_width(self, width_mm, duration=None):
        width_mm = float(width_mm)
        if not math.isfinite(width_mm) or not 0 <= width_mm <= self.MAX_WIDTH_MM:
            raise ValueError('Tool width is outside the supported range')
        self.move_to(1. - width_mm / self.MAX_WIDTH_MM)

    @property
    def is_moving(self):
        return self.state in (GripperState.OPENING, GripperState.CLOSING)

    @property
    def target_position(self):
        return None if self._target is None else (self._target - self.lower) / (self.upper - self.lower)

    @property
    def commanded_position(self):
        return self.target_position

    def observation(self):
        actual = self._read()
        return dict(actual_position_rad=actual.pos, actual_velocity_rad_s=actual.vel,
                    target_position=self.target_position, motion_status=self.state.value,
                    object_detection=self.object_detection, speed_fraction=self.speed_fraction,
                    force_setting=self.force_setting)
