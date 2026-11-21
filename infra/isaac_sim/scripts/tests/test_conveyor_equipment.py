"""Only the observed arrival can enter the configured supply."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_cold_start_and_expired_commands_hold_the_same_inventory(monkeypatch):
    from dataclasses import asdict
    from types import SimpleNamespace
    from conveyor_pnp_cell import runtime
    from conveyor_pnp_cell.equipment_io import Commands, Sensors
    from isaac_runtime.cell_supply import clear_supply_port
    now = 100.
    monkeypatch.setattr(runtime.time, 'monotonic', lambda: now)
    command = Commands(1, 10, True, False, False, False, False, 0, 0, 0, 0)
    packets, spawned, speeds = [], [], []
    def send_command(cmd):
        packets.append(dict(op='command', at=now, command=asdict(cmd), permitted=True, permit_at=now))
    def poll():
        batch = list(packets)
        packets.clear()
        return batch
    outgoing = []
    ipc = SimpleNamespace(available=True, send=outgoing.append, poll_messages=poll)

    def spawn(name, cycle):
        eq.cycle, eq.material_type_id = cycle, name
        spawned.append(cycle)
        return True

    eq = SimpleNamespace(stream_id='test', cycle=0, material_type_id='', drive=speeds.append, step=lambda *args: None,
                         observe=lambda permit: Sensors(permitted=permit, path_clear=True, pick_zone_clear=True),
                         spawn=spawn)
    adapter = runtime.ConveyorRuntime(dict(cell=dict(configured_supply=dict(material_id='ConveyorPart'),
        conveyor=dict(material_type_ids=['ConveyorPart'], pick_slot_id='cell/belt/pick', connection={}, equipment_permit_topic='permit', teaching=dict(cycle=None)))),
        eq, ipc)
    try:
        adapter.feeder.configure('ConveyorPart', 2)
        adapter.feeder.set_automatic(True)
        send_command(command)
        adapter.tick(.01)
        assert not adapter.synchronized and not spawned
        assert (outgoing[-1]['created_cycle'], outgoing[-1]['created_type_id']) == (0, '')
        send_command(Commands(0, 0, False, False, False, False, False, 0, 0, 0, 0))
        adapter.tick(.01)
        send_command(command)
        adapter.tick(.01)
        now += 2.
        adapter.tick(.01)
        assert speeds[-1] == 0
        send_command(command)
        adapter.tick(.01)
        assert spawned == [1] and adapter.feeder.remaining == 1
        assert (outgoing[-1]['created_cycle'], outgoing[-1]['created_type_id']) == (1, 'ConveyorPart')
    finally:
        clear_supply_port(adapter.feeder)


def test_suction_resolves_only_the_current_reserved_piece(monkeypatch):
    from types import SimpleNamespace
    from conveyor_pnp_cell import runtime

    adapter = runtime.ConveyorRuntime.__new__(runtime.ConveyorRuntime)
    adapter.stopped = False
    adapter.sidecar = SimpleNamespace(available=True)
    bodies = {('A', '12'): '/World/Part_12', ('B', '13'): '/World/Part_13'}
    adapter.equipment = SimpleNamespace(material_path=lambda part, instance: bodies.get((part, instance)))
    lease = dict(supply_id='conveyor_pick', slot_id='cell/belt/pick', supply_epoch='epoch',
                 reservation_id='second', material_id='ConveyorPart', material_type_id='B',
                 material_instance_id='13')
    adapter.packet = dict(at=100., reservation=lease)
    monkeypatch.setattr(runtime.time, 'monotonic', lambda: 100.)
    assert adapter.reserved_body(lease) == '/World/Part_13'
    for key, previous in (('material_instance_id', '12'), ('material_type_id', 'A'),
                          ('slot_id', 'cell/other/pick'), ('reservation_id', 'first'),
                          ('supply_epoch', 'old')):
        assert adapter.reserved_body(dict(lease, **{key: previous})) is None


def test_start_automatic_reconnects_the_existing_equipment_stream(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from conveyor_pnp_cell import runtime
    from isaac_runtime import cell_supply

    initial, replacement = Mock(available=True), Mock(available=True)
    launch = Mock(return_value=replacement)
    monkeypatch.setattr(runtime.ConfiguredSupplyClient, 'launch', launch)
    config = dict(cell=dict(configured_supply=dict(material_id='ConveyorPart'),
                            conveyor=dict(material_type_ids=['A'], pick_slot_id='cell/belt/pick',
                                          connection={}, equipment_permit_topic='permit',
                                          teaching=dict(cycle=None), history_db_path='/tmp/test-history')))
    equipment = SimpleNamespace(stream_id='stage', drive=Mock())
    adapter = runtime.ConveyorRuntime(config, equipment, initial)
    try:
        cell_supply.request_supply('A', 2)
        cell_supply.set_automatic(True)
        adapter.feeder.feed(1, lambda *_: True)
        adapter.synchronized = True
        initial.available = False
        adapter.packet = dict(op='command', reservation={'reservation_id': 'stale'})
        assert cell_supply.set_automatic(True)['accepted']
        assert adapter.sidecar is replacement
        initial.close.assert_called_once()
        packet = replacement.send.call_args.args[0]
        assert packet == dict(initial.send.call_args.args[0], resume_stream=True)
        assert packet['supply']['stream_id'] == equipment.stream_id
        assert adapter.packet is None and adapter.synchronized
        assert (adapter.feeder.last_generated, adapter.feeder.remaining) == (1, 1)
        replacement.available = False
        adapter.reset_generated_materials(None)
        assert not cell_supply.set_automatic(True)['accepted']
        launch.assert_called_once()
    finally:
        adapter.close()
