"""Correlate fresh Tool observations with one configured grasp and release."""
import logging
import re
import time
from gripper_runtime.grasp_attachment import GraspState, HoldingObservation, HoldingState

LOGGER = logging.getLogger(__name__)


def configured_reference(packet, strategy):
    keys = ('motion_goal_uuid', 'supply_epoch', 'reservation_id')
    uuid = r'[0-9a-fA-F]{32}|[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}'
    if not isinstance(packet, dict) or packet.get('strategy') != strategy or any(
            not isinstance(packet.get(k), str) or not re.fullmatch(uuid, packet[k]) for k in keys):
        return None
    fields = ('material_id', 'supply_id', 'slot_id')
    if 'material_instance_id' in packet or 'material_type_id' in packet:
        fields += ('material_type_id', 'material_instance_id')
    if any(not isinstance(packet.get(k), str) or not packet[k].strip() for k in fields):
        return None
    return {k: packet[k] for k in (*keys, *fields, 'strategy')}


class ConfiguredGraspManager:
    def __init__(self, gripper, *, tool, binding, **unused):
        self.gripper, self.tool, self.binding = gripper, tool, binding
        self.ready = True
        self.tcp_path = getattr(binding, 'tcp_path', '')
        self.capture_reference_id = None
        self._reference = None
        self._close_id = None
        self._release_reference = None
        self._confirmed_release = None
        self._opening = False
        self._sample_floor = 0
        self._last_sample = 0
        self._failure = None
        self._sequence = 1
        self._holding = HoldingObservation(HoldingState.UNKNOWN, time.monotonic(), 1)
        self.state = GraspState.EMPTY
        self.accepted_relation = None
        self.accepted_execution_id = None
        self._writer = None
        self._diagnostic_elapsed = 0.
        self._detailed_diagnostics = False
        self._diagnostic_samples = 0
        self._diagnostic_key = None
        self._reset_completed = False

    def _record(self, state):
        if state is not self._holding.state:
            self._sequence += 1
        self._holding = HoldingObservation(state, time.monotonic(), self._sequence)
        self.state = GraspState.GRASPED if state is HoldingState.HELD else GraspState.SEEKING

    def holding_observation(self):
        return self._holding

    def restore_holding_sequence(self, sequence):
        self._sequence = max(self._sequence, int(sequence))
        self._record(self._holding.state)
        return self._sequence

    def configure_selected_grasp_reference(self, packet):
        if packet.get('invalidate') is True:
            identity = packet.get('motion_goal_uuid')
            receipt_id = (self._confirmed_release or {}).get('execution_id')
            if identity in (self.capture_reference_id, self._close_id, receipt_id):
                self.capture_reference_id = None
                if identity == receipt_id and self._opening:
                    # Retire the ACK/selection while live contraction/contact
                    # observations continue to revalidate the release receipt.
                    self._reference = None
                    return False
                if packet.get('retire_pending') is not True or identity != self._close_id:
                    self._reference = None
                    self.cancel_tool_transaction()
                    self.gripper.cancel_motion()
            return False
        decoded = self.tool.capture.decode_reference(packet)
        if decoded is None:
            self.capture_reference_id = None
            self._reference = None
            self.cancel_tool_transaction()
            self.gripper.cancel_motion()
            return False
        try:
            strategy = self.tool.capture.select(decoded)
        except (ValueError, RuntimeError):
            self.capture_reference_id = None
            self._reference = None
            self.cancel_tool_transaction()
            self.gripper.cancel_motion()
            return False
        if decoded != self._reference:
            self.cancel_tool_transaction()
            self.gripper.cancel_motion()
        self.gripper.strategy = strategy
        self._reference = decoded
        self.capture_reference_id = decoded['motion_goal_uuid']
        return True

    def cancel_tool_transaction(self):
        self._release_reference = None
        self._confirmed_release = None
        self._close_id = None
        self._opening = False
        self.accepted_relation = self.accepted_execution_id = None
        self._sequence += 1
        self._record(HoldingState.UNKNOWN)

    def note_gripper_command_applied(self):
        self.cancel_tool_transaction()

    def arm_close_transaction(self, execution_id=None, command_generation=None):
        self.cancel_tool_transaction()
        self._failure = None
        requires_reference = getattr(self.tool.capture, 'requires_capture_reference', True)
        if not self.ready or (requires_reference and
                              (not self._reference or execution_id != self.capture_reference_id)):
            self._failure = f'{self.diagnostic_prefix}_reference_or_reserved_material_unavailable'
            self.gripper.cancel_motion()
            return False
        self._close_id = execution_id or ''
        self._diagnostic_samples = 0
        self._sample_floor = self.gripper.holding_observation().sequence
        return True

    @property
    def close_failure_reason(self):
        return self._failure

    def reject_close_transaction(self, reason):
        self.cancel_tool_transaction()
        self._failure = reason
        self.gripper.cancel_motion()

    def note_open_release_command_applied(self, generation, width_mm):
        reference = dict(self._reference) if self._reference is not None else None
        self.cancel_tool_transaction()
        self._release_reference = reference
        self._opening = True
        self._diagnostic_samples = 0
        self._sample_floor = self.gripper.holding_observation().sequence
        return True

    def update(self, dt):
        observation = self.gripper.holding_observation()
        state = HoldingState.UNKNOWN
        fresh_sample = observation.sequence > max(self._sample_floor, self._last_sample)
        self._last_sample = max(self._last_sample, observation.sequence)
        valid = self._reference is not None or not getattr(self.tool.capture, 'requires_capture_reference', True)
        if self.ready and fresh_sample and self._opening and observation.state is HoldingState.RELEASED:
            state = HoldingState.RELEASED
            if self._release_reference is not None:
                self._confirmed_release = {**self._release_reference,
                    'execution_id': self._release_reference['motion_goal_uuid']}
        elif self.ready and fresh_sample and self._close_id is not None and valid and observation.state is HoldingState.HELD:
            state = HoldingState.HELD
        if state is not HoldingState.HELD:
            self.accepted_relation = self.accepted_execution_id = None
        self._record(state)
        try:
            self._capture_diagnostic(dt)
        except Exception:
            self._writer = None
            try:
                LOGGER.warning('Configured grasp diagnostics unavailable')
            except Exception:
                pass
        return self.state

    def set_detailed_diagnostics_enabled(self, enabled):
        self._detailed_diagnostics = bool(enabled)

    def _capture_diagnostic(self, dt):
        reference = self._release_reference if self._opening else self._reference
        execution = (reference or {}).get('motion_goal_uuid')
        if self._writer is None or not execution:
            return
        key = (execution, self._opening, self._holding.state, self._failure)
        changed = key != self._diagnostic_key
        self._diagnostic_elapsed += max(0., dt)
        detailed = (self._detailed_diagnostics and self._diagnostic_samples < 128
                    and self._diagnostic_elapsed >= .1)
        if not changed and not detailed:
            return
        self._diagnostic_key = key
        record = dict(schema_version=1, record_type='physical_grasp',
            record_id=f'{self.diagnostic_prefix}-{time.monotonic_ns()}', observed_monotonic_ns=time.monotonic_ns(),
            execution_id=execution, reference=dict(reference),
            holding_state=self._holding.state.value, phase='release' if self._opening else 'capture',
            failure_reason=self._failure,
            coverage={'detailed_enabled':self._detailed_diagnostics,
                      'detail_sample_limit_per_command':128,
                      'detail_samples_submitted':self._diagnostic_samples})
        if detailed or (changed and self._detailed_diagnostics):
            if detailed:
                self._diagnostic_elapsed = 0.
                self._diagnostic_samples += 1
            record['coverage']['detail_samples_submitted'] = self._diagnostic_samples
            record['coverage']['detail_reason'] = 'state_change' if changed else 'periodic'
            record.update(self.diagnostics())
            record['coverage']['contacts'] = 'not_captured'
        self._writer.submit(record)

    def release_receipt(self):
        if (self.ready and self._confirmed_release is not None
                and self._holding.state is HoldingState.RELEASED and self._holding.fresh()):
            return dict(self._confirmed_release)
        return None

    def attachment_owned(self):
        return False

    def reconcile_stale_owned_joint(self):
        reconcile = getattr(self.binding, 'reconcile_stale_owned_joint', None)
        return True if reconcile is None else reconcile()

    def set_diagnostic_observers(self, writer=None, physx_observer=None):
        self._writer = writer

    def reset_for_simulation_stop(self):
        if self._reset_completed:
            return True
        self.cancel_tool_transaction()
        self.capture_reference_id = None
        self._reference = None
        reset_stopped = getattr(self.binding, 'reset_stopped_tool', None)
        if not callable(reset_stopped) or not reset_stopped(self.gripper):
            self.gripper.cancel_motion()
        self.ready = False
        self._reset_completed = True
        clear = getattr(self.binding, 'clear', None)
        if callable(clear): clear()
        return True

    def shutdown(self):
        self.reset_for_simulation_stop()
        self.ready = False
        shutdown = getattr(self.binding, 'shutdown', None)
        if callable(shutdown):
            shutdown()
        return True
