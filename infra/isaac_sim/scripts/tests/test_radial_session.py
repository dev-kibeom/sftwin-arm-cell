"""Operational radial transactions correlate reserved targets and measured Holding."""
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gripper_runtime.radial_manager import RadialCapture, RadialGraspManager
from gripper_runtime.grasp_attachment import HoldingObservation, HoldingState

PACKET = dict(motion_goal_uuid='a'*32, strategy='internal_radial', material_id='TapeRoll17',
              supply_id='tape_cartridge', slot_id='slot_0', supply_epoch='b'*32,
              reservation_id='c'*32, insertion_depth_m=.02, thickness_m=.017, inner_radius_m=.06)

class Geometry:
    def solve_bore(self, *args):
        return dict(grasp_position_m=.009, release_position_m=.0192)

class Binding:
    valid = True
    def resolve_target(self, packet): return '/material' if self.valid else None
    def relative_transform(self, path): return [1.,0.,0.,.1,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.]

class Actuator:
    strategy = None
    intent = None
    sample = HoldingObservation(HoldingState.UNKNOWN, 0., 1)
    def holding_observation(self): return self.sample
    def cancel_motion(self): self.intent = 'STOP'


def make():
    binding, actuator = Binding(), Actuator()
    capture = RadialCapture(dict(grasp_strategies=['internal_radial'], position_tolerance_m=.0002), Geometry())
    manager = RadialGraspManager(actuator, tool=SimpleNamespace(capture=capture), binding=binding)
    return manager, binding, actuator


def test_decoder_rejects_missing_identity_and_nonfinite_geometry():
    manager, _, _ = make()
    assert manager.tool.capture.decode_reference(PACKET) == PACKET
    for replacement in ({'reservation_id':''}, {'motion_goal_uuid':'bad'}, {'inner_radius_m':float('nan')}):
        assert manager.tool.capture.decode_reference({**PACKET, **replacement}) is None


def test_correlated_holding_needs_no_stage_identity_or_true_relative_pose():
    manager, binding, actuator = make()
    def forbidden(*args): raise AssertionError('Stage truth entered holding control')
    binding.resolve_target=binding.relative_transform=forbidden
    assert manager.configure_selected_grasp_reference(PACKET)
    assert not manager.arm_close_transaction('d'*32)
    assert manager.arm_close_transaction('a'*32)
    actuator.sample = HoldingObservation(HoldingState.HELD, 1., 2)
    manager.update(.01)
    assert manager.holding_observation().state is HoldingState.HELD
    assert manager.accepted_relation is None


def test_invalidation_and_release_do_not_reuse_prior_held():
    manager, _, actuator = make()
    manager.configure_selected_grasp_reference(PACKET)
    manager.arm_close_transaction('a'*32)
    manager.note_open_release_command_applied(2, None)
    actuator.sample = HoldingObservation(HoldingState.HELD, 1., 2)
    manager.update(.01)
    assert manager.holding_observation().state is HoldingState.UNKNOWN
    actuator.sample = HoldingObservation(HoldingState.RELEASED, 1., 3)
    manager.update(.01)
    assert manager.holding_observation().state is HoldingState.RELEASED
    manager.configure_selected_grasp_reference(dict(motion_goal_uuid='a'*32, invalidate=True))
    assert manager.capture_reference_id is None
    assert manager.attachment_owned() is False


def test_target_identity_requires_live_reservation_and_unique_body():
    from gripper_runtime.radial_binding import reserved_identity_matches, target_contact_forces
    attrs = {f'arm_cell:{key}': PACKET[key] for key in
             ('material_id','supply_id','slot_id','supply_epoch','reservation_id')}
    attrs['arm_cell:occupied'] = True
    assert reserved_identity_matches(attrs.get, PACKET)
    attrs['arm_cell:reservation_id'] = 'd'*32
    assert not reserved_identity_matches(attrs.get, PACKET)
    contacts = [dict(actor0='/target',actor1='/finger1',lost=False,
                     points=[dict(impulse=(.01,0.,0.))]),
                dict(actor0='/other',actor1='/finger2',lost=False,
                     points=[dict(impulse=(99.,0.,0.))])]
    assert target_contact_forces(contacts, '/target', ('/finger1','/finger2','/finger3'), .01) == [1.,0.,0.]
    assert target_contact_forces(None, '/target', ('/finger1','/finger2','/finger3'), .01) is None


def test_prior_transaction_observation_cannot_prove_new_close():
    manager, _, actuator = make()
    actuator.sample = HoldingObservation(HoldingState.HELD, 1., 20)
    manager.configure_selected_grasp_reference(PACKET)
    manager.arm_close_transaction('a'*32)
    manager.update(.01)
    assert manager.holding_observation().state is HoldingState.UNKNOWN


def test_retired_ack_can_still_be_invalidated_by_its_execution_identity():
    manager, _, actuator = make()
    manager.configure_selected_grasp_reference(PACKET)
    manager.arm_close_transaction('a'*32)
    manager.configure_selected_grasp_reference(dict(motion_goal_uuid='a'*32, invalidate=True, retire_pending=True))
    manager.configure_selected_grasp_reference(dict(motion_goal_uuid='a'*32, invalidate=True))
    assert manager._close_id is None
    assert actuator.intent == 'STOP'


def test_release_receipt_survives_reference_retirement_but_not_new_command():
    manager, _, actuator = make()
    manager.configure_selected_grasp_reference(PACKET)
    manager.arm_close_transaction('a'*32)
    manager.note_open_release_command_applied(2, None)
    actuator.sample = HoldingObservation(HoldingState.RELEASED, 1., 30)
    manager.update(.01)
    receipt = manager.release_receipt()
    assert receipt['reservation_id'] == 'c'*32
    assert receipt['execution_id'] == 'a'*32
    manager.configure_selected_grasp_reference(dict(motion_goal_uuid='a'*32, invalidate=True))
    actuator.sample = HoldingObservation(HoldingState.RELEASED, 1., 31)
    manager.update(.01)
    assert manager.release_receipt()['reservation_id'] == 'c'*32
    manager.cancel_tool_transaction()
    assert manager.release_receipt() is None


def test_stopped_reset_does_not_touch_destroyed_dynamic_control_handles():
    manager, binding, actuator = make()
    writes = []
    binding.reset_stopped_tool = lambda tool: writes.append('stopped_reset') or True
    actuator.cancel_motion = lambda: (_ for _ in ()).throw(RuntimeError('DC destroyed'))
    assert manager.reset_for_simulation_stop()
    assert writes == ['stopped_reset']
    assert not manager.ready
    assert manager.release_receipt() is None
    # A replacement bootstrap runs after the next Play, while this old manager
    # still has obsolete DC handles. Its shutdown must remain idempotent.
    assert manager.shutdown()
    assert writes == ['stopped_reset']



def test_radial_records_are_sparse_by_default_and_release_keeps_identity():
    manager, _, actuator = make()
    records=[]
    actuator.observation=lambda:{'position':.01}
    manager.set_diagnostic_observers(SimpleNamespace(submit=lambda record:records.append(record)))
    manager.configure_selected_grasp_reference(PACKET)
    manager.arm_close_transaction(PACKET['motion_goal_uuid'])
    for sequence in range(2,102):
        actuator.sample=HoldingObservation(HoldingState.HELD,1.,sequence)
        manager.update(.1)
    assert len(records)<10
    manager.note_open_release_command_applied(2,None)
    actuator.sample=HoldingObservation(HoldingState.RELEASED,1.,102)
    manager.update(.1)
    manager.configure_selected_grasp_reference(dict(motion_goal_uuid=PACKET['motion_goal_uuid'],invalidate=True))
    actuator.sample=HoldingObservation(HoldingState.RELEASED,1.,103)
    manager.update(.1)
    released=[r for r in records if r['holding_state']=='released']
    assert released and released[-1]['execution_id']==PACKET['motion_goal_uuid']
    assert released[-1]['reference']['reservation_id']==PACKET['reservation_id']
    assert all('actuator' not in r for r in records)


def test_selected_radial_details_end_after_bounded_command_window():
    manager, _, actuator=make()
    records=[]
    actuator.observation=lambda:{'position':.01}
    manager.set_diagnostic_observers(SimpleNamespace(submit=lambda record:records.append(record)))
    manager.set_detailed_diagnostics_enabled(True)
    manager.configure_selected_grasp_reference(PACKET)
    manager.arm_close_transaction(PACKET['motion_goal_uuid'])
    for sequence in range(2,502):
        actuator.sample=HoldingObservation(HoldingState.HELD,1.,sequence)
        manager.update(.1)
    details=[record for record in records if 'actuator' in record]
    assert 0<len(details)<500
    assert len(details)<=details[-1]['coverage']['detail_sample_limit_per_command']


def test_diagnostic_observation_failure_preserves_physical_holding():
    manager, _, actuator=make()
    manager.set_diagnostic_observers(SimpleNamespace(submit=lambda record:True))
    manager.set_detailed_diagnostics_enabled(True)
    def broken():raise RuntimeError('diagnostic observation unavailable')
    actuator.observation=broken
    manager.configure_selected_grasp_reference(PACKET)
    manager.arm_close_transaction(PACKET['motion_goal_uuid'])
    actuator.sample=HoldingObservation(HoldingState.HELD,1.,2)
    manager.update(.1)
    assert manager.holding_observation().state is HoldingState.HELD
    assert manager.holding_observation().state is HoldingState.HELD


def test_late_holding_loss_keeps_decision_values_after_periodic_budget():
    manager, _, actuator = make()
    records = []
    actuator.observation = lambda: {'primary_position_m': .01, 'target_m': .009}
    manager.set_diagnostic_observers(SimpleNamespace(submit=lambda record: records.append(record)))
    manager.set_detailed_diagnostics_enabled(True)
    manager.configure_selected_grasp_reference(PACKET)
    manager.arm_close_transaction(PACKET['motion_goal_uuid'])
    for sequence in range(2, 502):
        actuator.sample = HoldingObservation(HoldingState.HELD, 1., sequence)
        manager.update(.1)
    actuator.sample = HoldingObservation(HoldingState.UNKNOWN, 1., 502)
    manager.update(.1)
    assert records[-1]['holding_state'] == 'unknown'
    assert records[-1]['actuator']['primary_position_m'] == .01
    assert records[-1]['coverage']['contacts'] == 'not_captured'
