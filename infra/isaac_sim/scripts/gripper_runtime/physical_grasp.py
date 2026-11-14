"""Physical release and conservative holding observations; never creates joints."""
import math
import logging
import time

from gripper_runtime.grasp_attachment import GraspState, HoldingState, HoldingObservation


LOGGER = logging.getLogger(__name__)


class PhysicalGraspManager:
    """Estimate holding from selected aperture and sustained observed obstruction.

    This is motor-state inference, never material ground truth or force sensing.
    """
    def __init__(self, gripper, *, tool, stage=None, tcp_path=None, holding_settings=None, **unused):
        self.gripper, self.tool = gripper, tool
        self.tcp_path = tcp_path or ''
        self.ready = True
        self.state = GraspState.EMPTY
        self.capture_reference_id = None
        self._sequence = 1
        self._holding = HoldingObservation(HoldingState.UNKNOWN, time.monotonic(), 1)
        self._opening = False
        self._close_reference_id = None
        self._expected_width_mm = None
        self._reference_signature = None
        self._close_width_mm = None
        self._stationary_position = None
        self._holding_dwell = 0.0
        self.holding_settings = holding_settings
        self._close_transaction_failure_reason = None
        self._observer = None
        self._writer = None
        self._diagnostic_elapsed = 0.0
        self._diagnostic_failure_logged = False
        if stage is not None:
            from gripper_runtime.physical_contacts import PhysicalContactObserver
            try:
                self._observer = PhysicalContactObserver()
            except Exception as error:
                LOGGER.warning('Physical contact diagnostics unavailable: %s', error)
            # Left-over logical ownership must not carry into Physical Grasp.
            from gripper_runtime.grasp_joint import UsdFixedJointOwner
            if not UsdFixedJointOwner(stage, self.tcp_path).reconcile_stale():
                raise RuntimeError('Cannot remove stale logical grasp before Physical Grasp')

    def _record(self, state):
        if state is not self._holding.state:
            self._sequence += 1
        self._holding = HoldingObservation(state, time.monotonic(), self._sequence)

    def holding_observation(self):
        return self._holding

    def restore_holding_sequence(self, sequence):
        self._sequence = max(self._sequence, int(sequence))
        self._record(self._holding.state)
        return self._sequence

    def configure_selected_grasp_reference(self, packet):
        if isinstance(packet, dict) and packet.get('invalidate') is True:
            identity = packet.get('motion_goal_uuid')
            if identity in (self.capture_reference_id, self._close_reference_id):
                self.capture_reference_id = None
                self._expected_width_mm = None
                self._reference_signature = None
                if not (packet.get('retire_pending') is True and identity == self._close_reference_id):
                    self.cancel_tool_transaction()
            return False
        try:
            identity, values = packet['motion_goal_uuid'], packet['values']
            frame = packet['frame_id']
            values = tuple(float(v) for v in values)
            valid = (isinstance(identity, str) and bool(identity.strip())
                     and isinstance(frame, str) and bool(frame.strip())
                     and len(values) == 9 and all(math.isfinite(v) for v in values)
                     and abs(sum(v*v for v in values[3:7]) - 1.) <= 1e-6
                     and values[8] in (0., 1., 2., 4.)
                     and 0. < values[7] < self.gripper.MAX_WIDTH_MM)
            if not valid:
                raise ValueError('invalid selected reference')
        except (KeyError, TypeError, ValueError, OverflowError):
            self.capture_reference_id = None
            self._expected_width_mm = None
            self.cancel_tool_transaction()
            return False
        signature = (identity, frame, values)
        if signature != self._reference_signature:
            self.cancel_tool_transaction()
        self._reference_signature = signature
        self.capture_reference_id, self._expected_width_mm = identity, values[7]
        return True

    def note_gripper_command_applied(self):
        self._sequence += 1
        self.cancel_tool_transaction()

    def arm_close_transaction(self, execution_id=None, command_generation=None):
        self.cancel_tool_transaction()
        self._close_transaction_failure_reason = None
        if (not self.ready or not self.capture_reference_id
                or execution_id != self.capture_reference_id or self.holding_settings is None):
            self._close_transaction_failure_reason = 'physical_grasp_reference_or_runtime_unavailable'
            return False
        self._close_reference_id = self.capture_reference_id
        self._close_width_mm = self._expected_width_mm
        self.state = GraspState.SEEKING
        return True

    @property
    def close_failure_reason(self):
        return self._close_transaction_failure_reason

    def reject_close_transaction(self, reason):
        self._close_transaction_failure_reason = reason
        self.cancel_tool_transaction()

    def attachment_owned(self):
        return False

    def _estimate_holding(self, dt):
        if not self._close_reference_id or self._close_width_mm is None:
            return HoldingState.UNKNOWN
        try:
            observation = self.gripper.observation()
            pos, velocity = observation['actual_position_rad'], observation['actual_velocity_rad_s']
            target = observation['target_position_rad']
            epsilon = observation['position_tolerance_rad']
            valid = (observation['command_direction'] == 'closing'
                     and observation['object_detection'] == 'stopped_closing'
                     and all(math.isfinite(v) for v in (pos, velocity, target))
                     and target - pos > epsilon
                     and observation['lower_limit_rad'] + epsilon < pos < observation['upper_limit_rad'] - epsilon
                     and abs(velocity) <= observation['stopped_velocity_rad_s']
                     and self.tool.capture.width_matches_reference(
                         observation['actual_width_mm'], self._close_width_mm,
                         self.holding_settings['width_tolerance_mm']))
        except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError):
            valid = False
        if not valid:
            self._holding_dwell, self._stationary_position = 0.0, None
            return HoldingState.UNKNOWN
        if self._stationary_position is None or abs(pos - self._stationary_position) > self.holding_settings['held_position_tolerance_rad']:
            self._stationary_position, self._holding_dwell = pos, 0.0
        self._holding_dwell += max(0., float(dt))
        return (HoldingState.HELD if self._holding_dwell >= self.holding_settings['stopped_dwell_s']
                else HoldingState.UNKNOWN)

    def note_open_release_command_applied(self, generation, width_mm):
        self.cancel_tool_transaction()
        self._opening = True
        self._sequence += 1
        self._record(HoldingState.UNKNOWN)
        return True

    def cancel_tool_transaction(self):
        self.state = GraspState.SEEKING
        self._opening = False
        self._close_reference_id = None
        self._close_width_mm = None
        self._stationary_position = None
        self._holding_dwell = 0.0
        self._record(HoldingState.UNKNOWN)

    def update(self, dt):
        if self._observer is not None:
            self._observer.advance()
        if not self.ready:
            self._record(HoldingState.UNKNOWN)
        elif self._opening and self.tool.capture.release_complete(self.gripper, None):
            self.state = GraspState.EMPTY
            self._record(HoldingState.RELEASED)
        else:
            inferred = self._estimate_holding(dt)
            self.state = GraspState.GRASPED if inferred is HoldingState.HELD else GraspState.SEEKING
            self._record(inferred)
        self._diagnostic_elapsed += max(0., float(dt))
        if self._writer is not None and self._diagnostic_elapsed >= 0.1:
            self._diagnostic_elapsed = 0.0
            try:
                self._writer.submit(dict(
                    schema_version=1, record_type='physical_grasp',
                    record_id=f'physical-{time.monotonic_ns()}',
                    execution_id=self._close_reference_id or self.capture_reference_id,
                    observed_monotonic_ns=time.monotonic_ns(),
                    time_bases={'observed_monotonic_ns': 'host monotonic clock'},
                    coverage={'contacts': 'latest_physics_step_only'},
                    **self.diagnostics()))
            except Exception as error:
                if not self._diagnostic_failure_logged:
                    LOGGER.warning('Physical grasp diagnostics unavailable: %s', error)
                    self._diagnostic_failure_logged = True
        return self.state

    def reset_for_simulation_stop(self):
        self.cancel_tool_transaction()
        self.tool.cancel_motion()
        self.capture_reference_id = None
        self._expected_width_mm = None
        self._reference_signature = None
        if self._observer is not None:
            self._observer.clear()
        return True

    def shutdown(self):
        self.reset_for_simulation_stop()
        self.ready = False
        if self._observer is not None:
            self._observer.close()
        return True

    def set_diagnostic_observers(self, writer=None, physx_observer=None):
        self._writer = writer

    @property
    def attach_joint_path(self):
        return None

    @property
    def is_grasped(self):
        return self._holding.state is HoldingState.HELD

    def diagnostics(self):
        return dict(holding_state=self._holding.state.value, attach_joint_path=None,
                    holding_inference=dict(source='joint_motion_and_selected_aperture',
                        execution_id=self._close_reference_id,
                        expected_width_mm=self._close_width_mm,
                        stationary_duration_s=self._holding_dwell),
                    actuator=self.gripper.observation(),
                    contacts=self._observer.snapshot if self._observer else None)
