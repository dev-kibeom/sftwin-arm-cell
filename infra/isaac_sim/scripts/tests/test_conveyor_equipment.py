"""Only the observed, catalogued arrival can enter the configured supply."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from conveyor_pnp_cell.configuration import fixed_pick_slots


def test_one_fixed_teaching_distinct_arrivals_and_bounded_deployment():
    teaching = {'approach_motion': 'fixed_pick'}
    catalog = fixed_pick_slots(teaching, first=7, count=2)
    assert catalog == {'slot_7': teaching, 'slot_8': teaching}
    assert fixed_pick_slots(None, first=1, count=2) == {}


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
    eq = SimpleNamespace(cycle=0, drive=speeds.append, step=lambda *args: None,
                         observe=lambda permit: Sensors(permitted=permit, path_clear=True, pick_zone_clear=True),
                         spawn=lambda name, cycle: spawned.append(cycle) or True)
    adapter = runtime.ConveyorRuntime(dict(cell=dict(configured_supply=dict(material_id='ConveyorPart'),
        conveyor=dict(connection={}, equipment_permit_topic='permit')), motion=dict(configured_grasp_cycles_json='{}')),
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
