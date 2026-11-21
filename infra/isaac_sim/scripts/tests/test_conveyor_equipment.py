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
    ipc = SimpleNamespace(available=True, send=lambda _: None, poll_messages=poll)
    eq = SimpleNamespace(stream_id='test', cycle=0, drive=speeds.append, step=lambda *args: None,
                         observe=lambda permit: Sensors(permitted=permit, path_clear=True, pick_zone_clear=True),
                         spawn=lambda name, cycle: spawned.append(cycle) or True)
    adapter = runtime.ConveyorRuntime(dict(cell=dict(configured_supply=dict(material_id='ConveyorPart'),
        conveyor=dict(material_type_ids=['ConveyorPart'], pick_slot_id='cell/belt/pick', connection={}, equipment_permit_topic='permit', teaching=dict(cycle=None)))),
        eq, ipc)
    try:
        adapter.feeder.configure('ConveyorPart', 2)
        adapter.feeder.set_automatic(True)
        send_command(command)
        adapter.tick(.01)
        assert not adapter.synchronized and not spawned
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
