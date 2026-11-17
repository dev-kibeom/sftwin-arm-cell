"""Configured inventory consumes only a correlated, physically completed reservation."""
import sys
from pathlib import Path
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
    assert not supply.handle(complete, valid=True, completion_observed=lambda slot:False)['success']
    assert supply.snapshot()['completed'] == 0
    assert supply.handle(complete, valid=True, completion_observed=lambda slot:True)['success']
    assert supply.handle(complete, valid=True, completion_observed=lambda slot:True)['success']
    assert supply.snapshot()['completed'] == 1
    second = supply.handle(dict(request(0), supply_epoch=supply.epoch), valid=True, completion_observed=lambda slot:True)
    assert second['slot_id'] == 'slot_1'
    assert not supply.handle(complete, valid=True, completion_observed=lambda slot:True)['success']
    assert supply.snapshot()['completed'] == 1
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


def test_completion_requires_settled_real_pose_and_finite_measurements():
    from tape_roll_cell.cell_runtime import receiver_settled
    target = (.5,.3,.6)
    assert receiver_settled(target, (0,0,0), (0,0,0), (0,0,1), target)
    assert not receiver_settled((.52,.3,.6), (0,0,0), (0,0,0), (0,0,1), target)
    assert not receiver_settled(target, (.03,0,0), (0,0,0), (0,0,1), target)
    assert not receiver_settled(target, (0,0,0), (.3,0,0), (0,0,1), target)
    assert not receiver_settled(target, (0,0,0), (0,0,0), (1,0,0), target)
    assert not receiver_settled(target, (float('nan'),0,0), (0,0,0), (0,0,1), target)


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
