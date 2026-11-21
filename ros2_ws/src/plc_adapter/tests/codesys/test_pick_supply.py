"""Arrival and reservation identity on the Conveyor equipment side of the boundary."""
from pathlib import Path
import sys

from arm_cell_interfaces.srv import ConfiguredSupply
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[5] / 'infra/isaac_sim/scripts'))
from conveyor_pnp_cell.configured_supply import ConveyorPickSupply


def pick_supply(epoch, stream_id='test'):
    return ConveyorPickSupply(epoch, supply_id='conveyor_pick', slot_id='cell/belt/pick',
                              stream_id=stream_id, material_id='ConveyorPart', material_type_ids=['A', 'B'])


@pytest.mark.parametrize('command', [ConfiguredSupply.Request.COMPLETE, ConfiguredSupply.Request.ABANDON])
def test_release_removes_only_the_reserved_arrival_even_after_failure(command):
    supply = pick_supply('epoch')
    assert supply.expose_arrival(12, 'A')

    def call(command, *, slot='', reservation='', instance='test/12', material_type='A',
             placed=False, epoch='epoch', valid=True):
        return supply.handle(ConfiguredSupply.Request(
            command=command, supply_id='conveyor_pick', supply_epoch=epoch, slot_id=slot,
            reservation_id=reservation, material_instance_id=instance, material_type_id=material_type,
            material_placed=placed), ConfiguredSupply.Response(), valid=valid)

    reserve = ConfiguredSupply.Request.RESERVE
    lease = call(reserve)
    assert lease.success and lease.slot_id == 'cell/belt/pick'
    assert not call(reserve).success
    assert not supply.expose_arrival(12, 'A') and not supply.expose_arrival(13, 'B')
    assert not call(command, slot=lease.slot_id, reservation=lease.reservation_id,
                    placed=True, epoch='old').success
    assert call(command, slot=lease.slot_id, reservation=lease.reservation_id, placed=True, valid=False).success
    assert not supply.expose_arrival(12, 'A') and call(reserve).exhausted
    assert supply.expose_arrival(13, 'B')
    second = call(reserve)
    assert (second.material_type_id, second.material_instance_id) == ('B', 'test/13')
    assert not call(command, slot=second.slot_id, reservation=second.reservation_id, placed=True).success
    assert second.success and second.slot_id == 'cell/belt/pick'
    assert not call(command, slot=lease.slot_id, reservation=lease.reservation_id, placed=True).success
    assert call(ConfiguredSupply.Request.ABANDON, slot=second.slot_id,
                reservation=second.reservation_id, instance='test/13', material_type='B').success
    assert call(reserve).slot_id == 'cell/belt/pick'


def test_unreserved_arrival_loses_admission_if_the_pick_observation_is_lost():
    from threading import Lock
    from time import monotonic
    from types import SimpleNamespace
    from conveyor_pnp_cell.sidecar import Exchange

    exchange = Exchange.__new__(Exchange)
    exchange.lock = Lock()
    exchange.published_observation = None
    exchange.valid_at = monotonic()
    exchange.ledger = object()
    exchange.identity_conflict = False
    exchange.restored_unverified = False
    exchange.supply = pick_supply('epoch')
    states = []
    exchange.publisher = SimpleNamespace(publish=states.append)
    assert exchange.supply.expose_arrival(12, 'A')
    for observed in (True, False, True):
        exchange.at_pick = observed
        exchange.publish_supply()
        assert states[-1].valid == observed
        assert states[-1].slot_ids == ['cell/belt/pick']


def test_sidecar_binds_observed_arrivals_beyond_the_old_teaching_range(open_exchange):
    from dataclasses import asdict
    from time import monotonic
    from conveyor_pnp_cell.equipment_io import Sensors, Commands

    exchange = open_exchange(pick_configured=False)
    command = Commands(65537, 22, False, False, True, True, False, 0, 0, 0, 0)
    packet = dict(sensors=asdict(Sensors(pick=True, stopped=True, extended=True)),
                  body_instance_id='stream/65537', created_cycle=65537, created_type_id='A', material_type_id='A', observed=65536, valid=True, acquired=monotonic(), feeder={})
    exchange.sample(packet, command)
    assert exchange.io.publish.call_args.kwargs['bound_cycle'] == 0
    packet['observed'] = command.cycle
    exchange.sample(packet, command)
    assert exchange.io.publish.call_args.kwargs['bound_cycle'] == 0
    exchange.pick_configured = True
    exchange.sample(packet, command)
    assert exchange.io.publish.call_args.kwargs['bound_cycle'] == command.cycle
    request = reserve_request(exchange)
    lease = exchange.reserve(request, ConfiguredSupply.Response())
    assert lease.success and lease.slot_id == 'cell/belt/pick' and lease.material_instance_id == 'stream/65537'
    request.command = request.COMPLETE
    request.slot_id, request.reservation_id = lease.slot_id, lease.reservation_id
    request.material_type_id, request.material_instance_id = lease.material_type_id, lease.material_instance_id
    request.material_placed = True
    assert exchange.reserve(request, ConfiguredSupply.Response()).success
    exchange.sample(packet, command)
    assert exchange.io.publish.call_args.kwargs['bound_cycle'] == 0
    assert exchange.ledger.history('stream/65537')['reservations'][0]['status'] == 'RELEASED'


def test_first_piece_after_plc_reset_is_distinct_from_a_new_reservation_epoch():
    def occupant(epoch, stream):
        supply = pick_supply(epoch, stream)
        assert supply.expose_arrival(1, 'A')
        assert not supply.expose_arrival(1, 'B')
        return supply.state(valid=True).material_instance_id

    assert occupant('epoch-1', 'line-run-1') == occupant('epoch-2', 'line-run-1')
    assert occupant('epoch-2', 'line-run-2') != occupant('epoch-2', 'line-run-1')


@pytest.fixture
def open_exchange(tmp_path, monkeypatch):
    from unittest.mock import Mock
    from conveyor_pnp_cell.sidecar import Exchange

    # Drive acquisitions explicitly without opening a Modbus connection.
    monkeypatch.setattr(Exchange, 'run', lambda self: None)
    exchanges = []
    def open_one(**overrides):
        config = dict(connection=dict(host='localhost', port=1502, timeout_seconds=.1,
                                      unit_id=1, poll_seconds=.05),
                      pick_configured=True, permit_topic='permit',
                      history_db_path=tmp_path/'history.sqlite',
                      supply=dict(supply_id='pick', material_id='Part', slot_id='cell/belt/pick',
                                  material_type_ids=['A'], stream_id='stream'))
        exchange = Exchange(Mock(), Mock(), dict(config, **overrides))
        exchange.io = Mock()
        exchanges.append(exchange)
        return exchange
    yield open_one
    for exchange in exchanges:
        exchange.close()


def send_sample(exchange, *, cycle=1, phase=22, occupied=False, confirmed=None, body=None):
    from dataclasses import asdict
    from time import monotonic
    from conveyor_pnp_cell.equipment_io import Commands, Sensors

    at = monotonic()
    command = Commands(cycle, phase, False, False, True, not occupied if confirmed is None else confirmed, False, 0, 0, 0, 0)
    sensors = Sensors(pick=not occupied, extended=True, stopped=True, occupied=occupied)
    packet = dict(op='sample', at=at, acquired=at, valid=True, sensors=asdict(sensors),
                  observed=cycle if sensors.pick else 0, material_type_id='A' if sensors.pick else '',
                  body_instance_id=body if body is not None else f'stream/{cycle}',
                  created_cycle=cycle, created_type_id='A', feeder={})
    exchange.sent[at] = command
    exchange.ipc.poll_messages.return_value = [packet]
    exchange.io.commands.return_value = command
    if exchange.use_plc_test_epoch:
        exchange.io.commands_with_epoch.return_value = (command, exchange.test_epoch)
    exchange.poll()


def reserve_request(exchange):
    return ConfiguredSupply.Request(command=ConfiguredSupply.Request.RESERVE, supply_id='pick',
                                    supply_epoch=exchange.supply.epoch)


def test_history_write_failure_preserves_equipment_feedback(open_exchange, monkeypatch):
    import sqlite3

    exchange = open_exchange()
    with monkeypatch.context() as fault:
        def disk_full(*args):
            raise sqlite3.OperationalError('disk full')
        fault.setattr(exchange.ledger, 'record_plc', disk_full)
        send_sample(exchange, phase=40, occupied=True)
        assert exchange.io.publish.call_args.kwargs['sensors'].occupied
        assert any(call.args[0]['op'] == 'command' for call in exchange.ipc.send.call_args_list)
    send_sample(exchange, phase=40, occupied=True)
    assert exchange.ledger.history('stream/1')['plc']['phase'] == 40


def test_snapshot_failure_withholds_binding_but_publishes_sensors(open_exchange, monkeypatch):
    import sqlite3

    exchange = open_exchange()
    with monkeypatch.context() as fault:
        def disk_full(*args, **kwargs):
            raise sqlite3.OperationalError('disk full')
        fault.setattr(exchange.ledger, 'save_supply', disk_full)
        send_sample(exchange)
        feedback = exchange.io.publish.call_args.kwargs
        assert feedback['valid'] and feedback['sensors'].pick and feedback['bound_cycle'] == 0
        assert not exchange.reserve(reserve_request(exchange), ConfiguredSupply.Response()).success
    send_sample(exchange)
    assert exchange.io.publish.call_args.kwargs['bound_cycle'] == 1


@pytest.mark.parametrize('terminal', [ConfiguredSupply.Request.COMPLETE, ConfiguredSupply.Request.ABANDON])
def test_recovered_reservation_accepts_matching_release_at_tray(open_exchange, terminal):
    first = open_exchange()
    send_sample(first)
    lease = first.reserve(reserve_request(first), ConfiguredSupply.Response())
    assert lease.success
    restored = open_exchange(resume_stream=True)
    send_sample(restored, phase=40, occupied=True)
    assert not restored.reserve(reserve_request(restored), ConfiguredSupply.Response()).success
    req = ConfiguredSupply.Request(command=terminal, supply_id='pick', supply_epoch=lease.supply_epoch,
                                   slot_id=lease.slot_id, material_type_id=lease.material_type_id,
                                   material_instance_id=lease.material_instance_id,
                                   material_placed=True, reservation_id='wrong')
    assert not restored.reserve(req, ConfiguredSupply.Response()).success
    req.reservation_id = lease.reservation_id
    assert restored.reserve(req, ConfiguredSupply.Response()).success
    assert restored.ledger.history('stream/1')['reservations'][0]['status'] == 'RELEASED'
    restored.publish_supply()
    assert restored.publisher.publish.call_args.args[0].valid
    send_sample(restored, cycle=2)
    assert restored.reserve(reserve_request(restored), ConfiguredSupply.Response()).success


def test_resume_without_saved_identity_keeps_equipment_online_and_supply_invalid(open_exchange):
    exchange = open_exchange(resume_stream=True)
    send_sample(exchange)
    assert exchange.io.publish.call_args.kwargs['valid']
    assert exchange.io.publish.call_args.kwargs['bound_cycle'] == 0
    exchange.publish_supply()
    assert not exchange.publisher.publish.call_args.args[0].valid


def test_failed_modbus_ack_preserves_the_acquired_observation(open_exchange):
    exchange = open_exchange()
    exchange.io.publish.side_effect = OSError('feedback acknowledgement lost')
    send_sample(exchange, phase=40, occupied=True)
    assert exchange.ipc.send.call_args.args[0]['op'] == 'offline'
    assert exchange.ledger.history('stream/1')['plc']['phase'] == 40


def test_reservation_is_only_acknowledged_after_durable_write(open_exchange):
    import sqlite3
    from contextlib import closing

    exchange = open_exchange()
    send_sample(exchange)
    req = reserve_request(exchange)
    with closing(sqlite3.connect(exchange.ledger.path)) as busy:
        busy.execute('BEGIN IMMEDIATE')
        failed = exchange.reserve(req, ConfiguredSupply.Response())
        assert not failed.success and not failed.reservation_id
        busy.rollback()
    assert exchange.ledger.summary()['unresolved_reservations'] == 0
    assert exchange.reserve(req, ConfiguredSupply.Response()).success


def test_recovered_abandon_without_placement_waits_for_the_same_pick(open_exchange):
    first = open_exchange()
    send_sample(first)
    lease = first.reserve(reserve_request(first), ConfiguredSupply.Response())
    restored = open_exchange(resume_stream=True)
    abandon = ConfiguredSupply.Request(command=ConfiguredSupply.Request.ABANDON, supply_id='pick',
                                       supply_epoch=lease.supply_epoch, slot_id=lease.slot_id,
                                       reservation_id=lease.reservation_id, material_type_id=lease.material_type_id,
                                       material_instance_id=lease.material_instance_id)
    assert restored.reserve(abandon, ConfiguredSupply.Response()).success
    req = reserve_request(restored)
    send_sample(restored, cycle=2)
    assert not restored.reserve(req, ConfiguredSupply.Response()).success
    send_sample(restored)
    new_lease = restored.reserve(req, ConfiguredSupply.Response())
    assert new_lease.success and new_lease.material_instance_id == lease.material_instance_id
    assert new_lease.reservation_id != lease.reservation_id


def test_history_service_joins_archived_stream_to_its_own_live_receipt_owner(open_exchange, tmp_path):
    import json
    from arm_cell_interfaces.msg import ConfiguredTaskTrace
    from arm_cell_interfaces.srv import QueryConveyorHistory
    from arm_cell_plc_adapter.receipts import Receipts, State

    path = tmp_path / 'first-plc.sqlite'
    owner = Receipts(path)
    try:
        owner.observe_idle()
        receipt = owner.claim(1)
        first = open_exchange(receipt_file=str(path))
        send_sample(first, cycle=7)
        lease = first.reserve(reserve_request(first), ConfiguredSupply.Response())
        trace = ConfiguredTaskTrace(material_instance_id=lease.material_instance_id,
                                    reservation_id=lease.reservation_id, task_status=6, result_code=5)
        trace.task_goal_id.uuid = list(range(16))
        owner.update(1, State.ABORTED, 3, 2, trace=trace)
        next_stream = open_exchange(supply=dict(supply_id='pick', material_id='Part', slot_id='cell/belt/pick',
                                                material_type_ids=['A'], stream_id='other'),
                                    receipt_file=str(tmp_path / 'other.sqlite'))
        request = QueryConveyorHistory.Request(material_instance_id='stream/7')
        response = next_stream.query_history(request, QueryConveyorHistory.Response())
        assert response.success
        history = json.loads(response.json_result)
        attempt, = history['robot_execution']['attempts']
        assert history['robot_execution']['status'] == 'linked'
        assert attempt['command_id'] == 1 and attempt['mission_goal_uuid'] == receipt.goal_uuid.hex()
        assert attempt['task_goal_uuid'] == bytes(range(16)).hex()
        assert attempt['task_outcome']['material_placed']
        assert attempt['reservation_id'] == history['reservations'][0]['reservation_id']
    finally:
        owner.close()


def test_receipt_source_index_failure_does_not_block_supply(open_exchange, monkeypatch):
    import sqlite3
    from conveyor_pnp_cell.production_ledger import ProductionLedger

    def unavailable(*args):
        raise sqlite3.OperationalError('receipt index busy')
    monkeypatch.setattr(ProductionLedger, 'bind_receipt_source', unavailable)
    exchange = open_exchange(receipt_file='plc.sqlite')
    send_sample(exchange)
    assert exchange.reserve(reserve_request(exchange), ConfiguredSupply.Response()).success


def test_reserved_pick_observation_tracks_physical_location_during_robot_request(open_exchange):
    exchange = open_exchange()
    send_sample(exchange)
    lease = exchange.reserve(reserve_request(exchange), ConfiguredSupply.Response())
    assert lease.success
    for occupied in (False, True, False):
        send_sample(exchange, phase=30, confirmed=False, occupied=occupied)
        exchange.publish_supply()
        state = exchange.publisher.publish.call_args.args[0]
        assert state.valid and state.reservation_id == lease.reservation_id
        assert state.reserved_material_at_pick == (not occupied)


def test_restored_running_lease_requires_same_live_plc_and_body(open_exchange):
    first = open_exchange()
    send_sample(first)
    lease = first.reserve(reserve_request(first), ConfiguredSupply.Response())
    restored = open_exchange(resume_stream=True)
    restored.publish_supply()
    assert restored.publisher.publish.call_args.args[0].observation_pending
    send_sample(restored, phase=30, confirmed=False)
    restored.publish_supply()
    state = restored.publisher.publish.call_args.args[0]
    assert state.valid and state.reservation_id == lease.reservation_id
    assert not restored.reserve(reserve_request(restored), ConfiguredSupply.Response()).success
    send_sample(restored, phase=30, confirmed=False, body='stream/2')
    restored.publish_supply()
    state = restored.publisher.publish.call_args.args[0]
    assert not state.valid and not state.observation_pending


def test_complete_with_lost_reply_confirms_only_the_same_release(open_exchange):
    first = open_exchange()
    send_sample(first)
    lease = first.reserve(reserve_request(first), ConfiguredSupply.Response())
    req = ConfiguredSupply.Request(command=ConfiguredSupply.Request.COMPLETE, supply_id='pick',
                                   supply_epoch=lease.supply_epoch, slot_id=lease.slot_id,
                                   material_type_id='A', material_instance_id=lease.material_instance_id,
                                   material_placed=True, reservation_id=lease.reservation_id)
    assert first.reserve(req, ConfiguredSupply.Response()).success
    restored = open_exchange(resume_stream=True)
    assert restored.reserve(req, ConfiguredSupply.Response()).success
    req.material_instance_id = 'stream/other'
    assert not restored.reserve(req, ConfiguredSupply.Response()).success
    assert len(restored.ledger.history('stream/1')['reservations']) == 1


def test_stale_sample_cannot_refresh_admission_or_allocate_a_new_lease(open_exchange, monkeypatch):
    from conveyor_pnp_cell import sidecar
    exchange = open_exchange()
    send_sample(exchange)
    exchange.publish_supply()
    assert exchange.publisher.publish.call_args.args[0].valid
    exchange.publish_supply()
    assert exchange.publisher.publish.call_count == 1
    monkeypatch.setattr(sidecar.time, 'monotonic', lambda: exchange.valid_at + .5)
    exchange.publish_supply()
    state = exchange.publisher.publish.call_args.args[0]
    assert state.observation_pending and not state.valid
    assert not exchange.reserve(reserve_request(exchange), ConfiguredSupply.Response()).success


def test_equipment_uses_acknowledged_plc_epoch_and_preserves_previous_index(open_exchange, tmp_path):
    base = tmp_path / 'plc.sqlite'
    exchange = open_exchange(use_plc_test_epoch=True, receipt_file=str(base))
    exchange.test_epoch = 1
    send_sample(exchange)
    assert exchange.receipt_file == str(tmp_path / 'plc.test-1.sqlite')
    assert exchange.ledger.receipt_source('stream') == exchange.receipt_file
    exchange.test_epoch = 2
    send_sample(exchange)
    assert exchange.ipc.send.call_args.args[0] == dict(op='offline')
    reopened = open_exchange(use_plc_test_epoch=True, receipt_file=str(base), resume_stream=True)
    reopened.test_epoch = 2
    send_sample(reopened)
    assert reopened.ipc.send.call_args.args[0] == dict(op='offline')
    assert reopened.ledger.receipt_source('stream') == str(tmp_path / 'plc.test-1.sqlite')


def test_cold_plc_can_continue_only_after_the_last_robot_and_physical_cycle_closed(open_exchange, tmp_path):
    from arm_cell_plc_adapter.receipts import Receipts, State
    from arm_cell_interfaces.msg import ConfiguredTaskTrace
    path = tmp_path / 'plc.sqlite'
    exchange = open_exchange(receipt_file=str(path))
    owner = Receipts(path)
    try:
        owner.observe_idle()
        owner.claim(1)
        send_sample(exchange)
        lease = exchange.reserve(reserve_request(exchange), ConfiguredSupply.Response())
        trace = ConfiguredTaskTrace(material_instance_id=lease.material_instance_id,
                                    reservation_id=lease.reservation_id)
        owner.update(1, State.SUCCEEDED, 7, 2, trace=trace)
        def closed():
            return owner.discharged_command(owner.latest(), tmp_path / 'history.sqlite')
        assert not closed()
        request = reserve_request(exchange)
        request.command = request.COMPLETE
        request.material_type_id, request.material_instance_id = lease.material_type_id, lease.material_instance_id
        request.reservation_id, request.slot_id, request.material_placed = lease.reservation_id, lease.slot_id, True
        assert exchange.reserve(request, ConfiguredSupply.Response()).success
        from conveyor_pnp_cell.equipment_io import Commands
        exchange.ledger.record_plc(Commands(1, 0, False, False, False, False, False, 1, 4, 7, 2))
        assert not closed()
        exchange.ledger.record_discharge(1, 100, 200)
        assert closed()
        from unittest.mock import Mock
        from arm_cell_plc_adapter.node import PlcTaskAdapter
        adapter = PlcTaskAdapter.__new__(PlcTaskAdapter)
        adapter.initialize_attempt = None
        adapter.conveyor_test_stream = True
        adapter.receipts, adapter.receipt_epoch = owner, 0
        adapter.history_path = tmp_path / 'history.sqlite'
        adapter.admission = None
        adapter.plc = Mock()
        adapter.plc.read_conveyor_command.return_value = (0, 0)
        adapter.exchange()
        assert adapter.plc.write_receipt.call_args.kwargs['conveyor_feedback'] == (0, 0, 0, 1, 0)
        assert adapter.plc.write_receipt.call_args.args[0].command_id == 1
        owner.update(1, State.UNCERTAIN)
        assert not closed()
        owner.update(1, State.SUCCEEDED, 7, 2)
        exchange.ledger.record_created(2, 'A')
        assert not closed()
    finally:
        owner.close()


def test_acquired_reservation_reaches_kit_before_slow_plc_reply(open_exchange, monkeypatch):
    from conveyor_pnp_cell import sidecar
    clock = [100.]
    monkeypatch.setattr(sidecar.time, 'monotonic', lambda: clock[0])
    exchange = open_exchange()
    send_sample(exchange)
    lease = exchange.reserve(reserve_request(exchange), ConfiguredSupply.Response())
    assert lease.success
    exchange.ipc.send.reset_mock()

    def slow_reply(**_feedback):
        packet = exchange.ipc.send.call_args.args[0]
        assert packet['op'] == 'reservation'
        assert packet['reservation']['reservation_id'] == lease.reservation_id
        assert packet['reservation']['material_instance_id'] == lease.material_instance_id
        clock[0] += .6

    exchange.io.publish.side_effect = slow_reply
    send_sample(exchange, phase=30)
    assert not exchange.valid()
    assert any(call.args[0]['op'] == 'command' for call in exchange.ipc.send.call_args_list)
    command_packet = next(call.args[0] for call in exchange.ipc.send.call_args_list if call.args[0]['op'] == 'command')
    assert 'reservation' not in command_packet


def test_equipment_reads_command_and_epoch_from_one_plc_scan():
    from unittest.mock import Mock
    from conveyor_pnp_cell.equipment_io import ConveyorIO
    words = [0] * 32
    words[17:19] = [1, 30]
    words[29:] = [0xC051, 0x1A18, 9]
    client = Mock()
    reply = client.read_input_registers.return_value
    reply.isError.return_value = False
    reply.function_code, reply.registers = 4, words
    command, epoch = ConveyorIO(client).commands_with_epoch()
    assert (command.cycle, command.phase, epoch) == (1, 30, 9)
    client.read_input_registers.assert_called_once_with(0, 32, unit=1)


def test_robot_request_keeps_verified_pick_slot_reservable_after_confirmation_pulse(open_exchange):
    exchange = open_exchange()
    send_sample(exchange, phase=30, confirmed=False)
    assert not exchange.supply.available
    send_sample(exchange, phase=22, confirmed=True)
    send_sample(exchange, phase=30, confirmed=False, body='another/1')
    assert not exchange.reserve(reserve_request(exchange), ConfiguredSupply.Response()).success
    send_sample(exchange, phase=30, confirmed=False)
    exchange.publish_supply()
    assert exchange.publisher.publish.call_args.args[0].valid
    lease = exchange.reserve(reserve_request(exchange), ConfiguredSupply.Response())
    assert lease.success and lease.material_instance_id == 'stream/1'
