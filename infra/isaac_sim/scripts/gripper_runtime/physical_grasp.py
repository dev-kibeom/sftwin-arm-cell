"""Physical release and conservative holding observations; never creates joints."""
import logging
import time
from types import SimpleNamespace

from gripper_runtime.grasp_attachment import GraspState, HoldingState, HoldingObservation


LOGGER = logging.getLogger(__name__)


class PhysicalGraspManager:
    """An obstruction/contact is insufficient evidence of load retention.

    This implementation intentionally publishes UNKNOWN on CLOSE. It provides
    physical interaction for inspection without inventing successful PnP evidence.
    """
    def __init__(self, gripper, *, tool, stage=None, tcp_path=None, **unused):
        self.gripper, self.tool = gripper, tool
        self.tcp_path = tcp_path or ''
        self.candidate_source = SimpleNamespace(require_motion_reference=False)
        self.ready = True
        self.state = GraspState.EMPTY
        self.capture_reference_id = None
        self.capture_diagnostic_record = None
        self.capture_diagnostic_target_path = None
        self._sequence = 1
        self._holding = HoldingObservation(HoldingState.UNKNOWN, time.monotonic(), 1)
        self._opening = False
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
        self.capture_reference_id = None if packet.get('invalidate') else packet.get('motion_goal_uuid')
        return True

    def note_gripper_command_applied(self):
        self._sequence += 1
        self._record(HoldingState.UNKNOWN)

    def arm_close_transaction(self, execution_id=None, command_generation=None):
        self._opening = False
        self.state = GraspState.SEEKING
        self._record(HoldingState.UNKNOWN)
        return self.ready

    def note_open_release_command_applied(self, generation, width_mm):
        self._opening = True
        self._sequence += 1
        self._record(HoldingState.UNKNOWN)
        return True

    def cancel_tool_transaction(self):
        self._opening = False
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
            # Do not turn motor stall or finger contact into material ownership.
            self._record(HoldingState.UNKNOWN)
        self._diagnostic_elapsed += max(0., float(dt))
        if self._writer is not None and self._diagnostic_elapsed >= 0.1:
            self._diagnostic_elapsed = 0.0
            try:
                self._writer.submit(dict(
                    schema_version=1, record_type='physical_grasp',
                    record_id=f'physical-{time.monotonic_ns()}',
                    execution_id=self.capture_reference_id,
                    observed_monotonic_ns=time.monotonic_ns(),
                    time_bases={'observed_monotonic_ns': 'host monotonic clock'},
                    coverage={'contacts': 'latest_physics_step_only'},
                    **self.diagnostics()))
            except Exception as error:
                if not self._diagnostic_failure_logged:
                    LOGGER.warning('Physical grasp diagnostics unavailable: %s', error)
                    self._diagnostic_failure_logged = True
        return self.state

    def reconcile_stale_owned_joint(self):
        return True

    def reset_for_simulation_stop(self):
        self.cancel_tool_transaction()
        self.tool.cancel_motion()
        self.capture_reference_id = None
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

    def consume_release_detach_event(self):
        return None

    @property
    def attach_joint_path(self):
        return None

    @property
    def is_grasped(self):
        return False

    def diagnostics(self):
        return dict(holding_state=self._holding.state.value, attach_joint_path=None,
                    actuator=self.gripper.observation(),
                    contacts=self._observer.snapshot if self._observer else None)
