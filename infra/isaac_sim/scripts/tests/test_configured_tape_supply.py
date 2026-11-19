"""Configured inventory consumes only a correlated, physically completed reservation."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.supply import ConfiguredTapeSupply


def request(command, **kw):
    return dict(command=command, supply_id='tape_cartridge', supply_epoch='',
                reservation_id='', slot_id='', material_placed=False, **kw)


def test_stale_token_failure_and_repeated_completion_preserve_exact_quantity():
    supply = ConfiguredTapeSupply(2)
    reserved = supply.handle(dict(request(0), supply_epoch=supply.epoch), valid=True, completion_observed=lambda slot:False)
    token = {k:reserved[k] for k in ('supply_epoch','reservation_id','slot_id')}
    complete = request(1)
    complete.update(token, material_placed=True)
    assert not supply.handle(dict(complete,material_placed=False), valid=True)['success']
    assert supply.snapshot()['completed'] == 0
    assert supply.handle(complete, valid=True, completion_observed=lambda slot:True)['success']
    assert supply.handle(complete, valid=True, completion_observed=lambda slot:True)['success']
    assert supply.snapshot()['completed'] == 0
    second = supply.handle(dict(request(0), supply_epoch=supply.epoch), valid=True, completion_observed=lambda slot:True)
    assert second['slot_id'] == 'slot_1'
    assert not supply.handle(complete, valid=True, completion_observed=lambda slot:True)['success']
    assert supply.snapshot()['completed'] == 0
    old_epoch = supply.epoch
    supply.reset()
    assert supply.epoch != old_epoch
    assert not supply.handle(complete, valid=True, completion_observed=lambda slot:True)['success']
    assert supply.snapshot()['completed'] == 0


def test_abandon_requires_exact_token_and_never_consumes():
    supply = ConfiguredTapeSupply(1)
    reserved = supply.handle(dict(request(0), supply_epoch=supply.epoch), valid=True, completion_observed=lambda slot:True)
    abandon = request(2)
    abandon.update({k:reserved[k] for k in ('supply_epoch','reservation_id','slot_id')})
    assert supply.handle(abandon, valid=False, completion_observed=lambda slot:True)['success']
    assert supply.snapshot()['completed'] == 0
    assert supply.handle(dict(request(0), supply_epoch=supply.epoch), valid=False, completion_observed=lambda slot:True)['success'] is False


def test_cycle_completion_does_not_query_material_truth_or_count_transferred_parts():
    supply = ConfiguredTapeSupply(1)
    def forbidden(*args): raise AssertionError('material GT used for completion')
    reserved = supply.handle(dict(request(0), supply_epoch=supply.epoch), valid=True)
    complete=dict(request(1), **{k:reserved[k] for k in ('supply_epoch','reservation_id','slot_id')})
    complete['material_placed']=True
    assert supply.handle(complete, valid=True, completion_observed=forbidden)['success']
    assert supply.snapshot()['exhausted']
    assert supply.state(valid=True)['completed_count']==0


def test_socket_relay_preserves_request_correlation_and_state():
    import socket
    from tape_roll_cell.configured_supply_sidecar import ConfiguredSupplyClient
    a,b = socket.socketpair()
    client = ConfiguredSupplyClient(a)
    b.sendall(b'{"op":"request","correlation_id":"test","request":{"command":0}}\n')
    packets = client.poll_messages()
    assert packets[0]['correlation_id'] == 'test'
    client.send(dict(op='response',correlation_id='test',response={'success':True}))
    import json
    assert json.loads(b.recv(4096))['response']['success']
    b.close()
    client.poll_messages()
    assert not client.available
    client.close()


def test_release_evidence_cannot_cross_reservation_or_epoch():
    import time
    from types import SimpleNamespace
    from gripper_runtime.grasp_attachment import HoldingObservation, HoldingState
    from tape_roll_cell.cell_runtime import ConfiguredSupplyAdapter
    supply = ConfiguredTapeSupply(1)
    reserved = supply.handle(dict(request(0), supply_epoch=supply.epoch), valid=True, completion_observed=lambda slot:False)
    reference = dict(reserved, supply_id=supply.supply_id, material_id=supply.material_id, execution_id='a'*32)
    manager = SimpleNamespace(release_receipt=lambda:dict(reference),
        holding_observation=lambda:HoldingObservation(HoldingState.RELEASED,time.monotonic(),1))
    adapter = ConfiguredSupplyAdapter.__new__(ConfiguredSupplyAdapter)
    adapter.supply, adapter.tool_session = supply, SimpleNamespace(grasp_manager=manager)
    assert adapter._release_observed(0)
    reference['reservation_id'] = 'wrong'
    assert not adapter._release_observed(0)
    reference.update(reserved)
    reference['supply_epoch'] = 'old'
    assert not adapter._release_observed(0)
    manager.holding_observation = lambda:HoldingObservation(HoldingState.UNKNOWN,time.monotonic(),2)
    reference.update(reserved)
    assert not adapter._release_observed(0)


def test_reserve_rejects_previous_session_and_active_lease():
    supply = ConfiguredTapeSupply(3)
    old_epoch = supply.epoch
    supply.reset()
    assert not supply.handle(dict(request(0), supply_epoch=old_epoch), valid=True,
                             completion_observed=lambda _:False)['success']
    assert supply.snapshot()['selected'] is None
    first = supply.handle(dict(request(0), supply_epoch=supply.epoch), valid=True,
                          completion_observed=lambda _:False)
    assert first['success']
    second = supply.handle(dict(request(0), supply_epoch=supply.epoch), valid=True,
                           completion_observed=lambda _:False)
    assert not second['success']
    assert supply.reservation_id == first['reservation_id']
    assert supply.snapshot()['completed'] == 0


def test_empty_infeed_is_not_exhaustion_or_fifteen_completed_rolls():
    supply=ConfiguredTapeSupply(15)
    epoch=supply.epoch
    supply.unload()
    assert supply.snapshot()==dict(occupied=[False]*15,completed=0,selected=None,exhausted=False)
    supply.reset()
    assert supply.epoch!=epoch
    assert supply.snapshot()['occupied']==[True]*15
    assert supply.snapshot()['completed']==0


def test_mission_advice_waits_for_published_valid_epoch():
    from tape_roll_cell.configured_supply_sidecar import supply_announced_for_mission
    state=dict(valid=True,supply_epoch='new-cartridge')
    assert not supply_announced_for_mission(state,True,None)
    assert not supply_announced_for_mission(state,True,'old-cartridge')
    assert supply_announced_for_mission(state,True,'new-cartridge')
    assert not supply_announced_for_mission(state,False,'new-cartridge')
    assert not supply_announced_for_mission(dict(state,valid=False),True,'new-cartridge')


def test_predictive_contact_reports_are_distinct_from_physical_contact():
    from tape_roll_cell.cell_runtime import physical_contact_observed
    def report(separation, impulse=(0., 0., 0.)):
        return {'points': [{'separation_m': separation, 'impulse': impulse}]}
    assert not physical_contact_observed(report(.0015587))
    assert physical_contact_observed(report(0.))
    assert physical_contact_observed(report(-.00001))
    assert physical_contact_observed(report(.001, (.000001, 0., 0.)))
    assert physical_contact_observed({'points': []})
    assert physical_contact_observed({'points': [{'impulse': (0., 0., 0.)}]})
    assert physical_contact_observed(report(float('nan')))
    assert physical_contact_observed(report(.001, (float('nan'), 0., 0.)))
    assert physical_contact_observed({'points': [
        report(.001)['points'][0], report(-.00001)['points'][0]]})


def test_collision_diagnostics_never_invalidate_supply_or_abort_control():
    from types import SimpleNamespace as S
    from tape_roll_cell.cell_runtime import ConfiguredSupplyAdapter
    adapter=ConfiguredSupplyAdapter.__new__(ConfiguredSupplyAdapter)
    adapter.conveyor=S(on_physics_step=lambda dt:None)
    adapter.timeline=S(is_playing=lambda:True)
    adapter.observer=S(advance=lambda:None,snapshot=[dict(actor0='/robot',actor1='/roll',lost=False,points=[])])
    adapter.failure='';adapter.tool_session=S(grasp_manager=None)
    adapter.paths=['/roll'];adapter.supply=S(_selected=None)
    adapter.context={'root_path':'/cell'};adapter.robot_bodies={'/robot'}
    from tape_roll_cell.cycle_evidence import CycleEvidence
    records=[];adapter._cycle_evidence=CycleEvidence(records.append)
    adapter._cycle_evidence.start('cycle')
    adapter._observe_cycle_contacts()
    assert adapter.failure==''
    adapter._cycle_evidence.finish(True)
    assert records[-1]['verification']=='FAIL'
    assert records[-1]['sequence_completed'] is True


def test_contact_observer_is_drained_without_an_active_cycle():
    from types import SimpleNamespace as S
    from tape_roll_cell.cell_runtime import ConfiguredSupplyAdapter
    adapter=ConfiguredSupplyAdapter.__new__(ConfiguredSupplyAdapter)
    adapter.conveyor=S(on_physics_step=lambda dt:None)
    adapter.timeline=S(is_playing=lambda:True)
    drained=[];adapter.observer=S(advance=lambda:drained.append(True),snapshot=[])
    adapter._cycle_evidence=None
    adapter._observe_cycle_contacts()
    assert drained


def test_missing_contact_observer_marks_verification_unavailable_without_stopping():
    from types import SimpleNamespace as S
    from tape_roll_cell.cell_runtime import ConfiguredSupplyAdapter
    from tape_roll_cell.cycle_evidence import CycleEvidence
    adapter=ConfiguredSupplyAdapter.__new__(ConfiguredSupplyAdapter)
    adapter.conveyor=S(on_physics_step=lambda dt:None)
    adapter.timeline=S(is_playing=lambda:True)
    adapter.observer=None;adapter.failure=''
    records=[];adapter._cycle_evidence=CycleEvidence(records.append)
    adapter._cycle_evidence.start('cycle')
    adapter._observe_cycle_contacts()
    adapter._cycle_evidence.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'
    assert adapter.failure==''


def test_pending_terminal_contact_is_included_before_cycle_verdict():
    from types import SimpleNamespace as S
    from tape_roll_cell.cell_runtime import ConfiguredSupplyAdapter
    from tape_roll_cell.cycle_evidence import CycleEvidence
    adapter=ConfiguredSupplyAdapter.__new__(ConfiguredSupplyAdapter)
    records=[];adapter._cycle_evidence=CycleEvidence(records.append)
    adapter._cycle_evidence.start('cycle')
    adapter._cycle_evidence.physics_step(1)
    adapter._cycle_evidence.observe([], step_id=1)
    adapter._pending_cycle='cycle';adapter._cycle_slot=0
    adapter.paths=['/roll'];adapter.supply=S(_selected=None)
    adapter.robot_bodies={'/robot'};adapter.tool_session=None
    adapter.context={'root_path':'/cell'};adapter.failure=''
    adapter._last_state_at=float('inf');adapter.teaching=S(tick=lambda:None)
    adapter.conveyor=S(release_to_outfeed=lambda path:None)
    adapter.observer=S(advance=lambda:None,snapshot=[dict(actor0='/robot',actor1='/fixture',lost=False,points=[])])
    adapter.sidecar=S(poll_messages=lambda:[dict(op='mission_result',request_id='cycle',success=True)],
                      flush_outgoing=lambda:None)
    adapter._poll_sidecar()
    assert adapter.failure==''
    assert records[-1]['verification']=='FAIL'
    assert records[-1]['observed_steps']==1


@pytest.mark.parametrize('observe_tail,expected', [(False,'UNAVAILABLE'),(True,'PASS')])
def test_cycle_coverage_does_not_change_device_or_mission_progress(observe_tail, expected):
    from types import SimpleNamespace as S
    from tape_roll_cell.cell_runtime import ConfiguredSupplyAdapter
    from tape_roll_cell.cycle_evidence import CycleEvidence
    adapter=ConfiguredSupplyAdapter.__new__(ConfiguredSupplyAdapter)
    records=[];adapter._cycle_evidence=CycleEvidence(records.append)
    adapter._cycle_evidence.start('cycle')
    adapter._physics_step_sequence=0
    commands=[];adapter.conveyor=S(on_physics_step=commands.append,
                                 release_to_outfeed=lambda path:commands.append('outfeed'))
    adapter.timeline=S(is_playing=lambda:True)
    adapter.observer=S(advance=lambda:None,snapshot=[])
    adapter.robot_bodies=set();adapter.paths=['/roll'];adapter._cycle_slot=0
    adapter.supply=S(_selected=None);adapter.tool_session=None
    adapter._pending_cycle='cycle';adapter.failure=''
    adapter._last_state_at=float('inf');adapter.teaching=S(tick=lambda:None)
    adapter.sidecar=S(poll_messages=lambda:[dict(op='mission_result',request_id='cycle',success=True)],
                      flush_outgoing=lambda:None)
    adapter._on_physics_step(.01);adapter._on_physics_observation(.01)
    # Physics and device commands continue while the observation callback stops.
    for _ in range(2):
        adapter._on_physics_step(.01)
        if observe_tail:
            adapter._on_physics_observation(.01)
    adapter._poll_sidecar()
    assert commands==[.01,.01,.01,'outfeed']
    assert adapter.failure=='' and adapter._pending_cycle is None
    assert records[-1]['sequence_completed'] is True
    assert records[-1]['verification']==expected
