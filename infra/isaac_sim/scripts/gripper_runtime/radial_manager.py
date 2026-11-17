"""Correlate radial contact Holding with a reserved material, without attachment."""
import math
import logging
import re
import time
from gripper_runtime.grasp_attachment import GraspState, HoldingObservation, HoldingState
from gripper_runtime.grasp_strategy import select_grasp_strategy


LOGGER = logging.getLogger(__name__)


class RadialCapture:
    name = 'internal_radial'

    def __init__(self, settings, geometry):
        self.settings, self.geometry = settings, geometry
        self.strategy = None

    @staticmethod
    def decode_reference(packet):
        try:
            if packet['strategy'] != 'internal_radial':
                return None
            for key in ('motion_goal_uuid', 'supply_epoch', 'reservation_id'):
                if not isinstance(packet[key], str) or not re.fullmatch(r'[0-9a-fA-F]{32}|[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}', packet[key]):
                    return None
            for key in ('material_id', 'supply_id', 'slot_id'):
                if not isinstance(packet[key], str) or not packet[key].strip():
                    return None
            for key in ('insertion_depth_m', 'thickness_m', 'inner_radius_m'):
                value = packet[key]
                if isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0:
                    return None
            return {key: packet[key] for key in ('motion_goal_uuid', 'strategy', 'material_id',
                'supply_id', 'slot_id', 'supply_epoch', 'reservation_id',
                'insertion_depth_m', 'thickness_m', 'inner_radius_m')}
        except (KeyError, TypeError, ValueError, OverflowError):
            return None

    def select(self, packet):
        self.strategy = select_grasp_strategy(self.settings, packet, geometry=self.geometry)
        return self.strategy


class RadialGraspManager:
    def __init__(self, gripper, *, tool, binding, **unused):
        self.gripper, self.tool, self.binding = gripper, tool, binding
        self.ready = True
        self.tcp_path = getattr(binding, 'tcp_path', '')
        self.capture_reference_id = None
        self._reference = None
        self._close_id = None
        self._release_reference = None
        self._confirmed_release = None
        self._target_path = None
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
            target = self.binding.resolve_target(decoded)
            if not target:
                raise ValueError('Reserved material is unavailable')
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
        self._reference, self._target_path = decoded, target
        self.capture_reference_id = decoded['motion_goal_uuid']
        self.binding.target_reference = decoded
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
        if (not self.ready or not self._reference or execution_id != self.capture_reference_id
                or self.binding.resolve_target(self._reference) != self._target_path):
            self._failure = 'radial_reference_or_reserved_material_unavailable'
            self.gripper.cancel_motion()
            return False
        self._close_id = execution_id
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
        self._sample_floor = self.gripper.holding_observation().sequence
        return True

    def update(self, dt):
        observation = self.gripper.holding_observation()
        state = HoldingState.UNKNOWN
        fresh_sample = observation.sequence > max(self._sample_floor, self._last_sample)
        self._last_sample = max(self._last_sample, observation.sequence)
        valid = (self._reference is not None and
                 self.binding.resolve_target(self._reference) == self._target_path)
        if self.ready and fresh_sample and self._opening and observation.state is HoldingState.RELEASED:
            state = HoldingState.RELEASED
            if self._release_reference is not None:
                self._confirmed_release = {**self._release_reference,
                    'execution_id': self._release_reference['motion_goal_uuid']}
        elif self.ready and fresh_sample and self._close_id and valid and observation.state is HoldingState.HELD:
            if self.accepted_relation is None:
                relation = self.binding.relative_transform(self._target_path)
                if len(relation) == 16 and all(math.isfinite(v) for v in relation):
                    self.accepted_relation = relation
                    self.accepted_execution_id = self._close_id
            if self.accepted_relation is not None:
                state = HoldingState.HELD
        if state is not HoldingState.HELD:
            self.accepted_relation = self.accepted_execution_id = None
        self._record(state)
        self._diagnostic_elapsed += max(0., dt)
        if self._writer is not None and self._diagnostic_elapsed >= .1:
            self._diagnostic_elapsed = 0.
            try:
                self._writer.submit(dict(schema_version=1, record_type='physical_grasp',
                    record_id=f'radial-{time.monotonic_ns()}', observed_monotonic_ns=time.monotonic_ns(),
                    execution_id=self._close_id or self.capture_reference_id,
                    coverage={'contacts':'latest_physics_step_only'}, **self.diagnostics()))
            except Exception as error:
                LOGGER.warning('Radial diagnostics unavailable: %s', error)
                self._writer = None
        return self.state

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

    def diagnostics(self):
        return dict(holding_state=self._holding.state.value, attach_joint_path=None,
                    holding_inference={'source':'three_reserved_target_contacts_and_obstruction'},
                    accepted_relation=self.accepted_relation, actuator=self.gripper.observation())

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
        close = getattr(self.binding, 'close', None)
        if callable(close): close()
        return True
