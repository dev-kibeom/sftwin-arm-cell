"""Only the observed arrival can enter the configured supply."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_cold_start_and_expired_commands_hold_the_same_inventory(monkeypatch, tmp_path):
    import json
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
                         spawn=spawn, material_path=lambda *_: None)
    adapter = runtime.ConveyorRuntime(dict(cell=dict(configured_supply=dict(material_id='ConveyorPart'),
        conveyor=dict(material_type_ids=['ConveyorPart'], pick_slot_id='cell/belt/pick', connection={}, equipment_permit_topic='permit', teaching=dict(cycle=None)))),
        eq, ipc)
    # A pre-existing evidence file causes a writer failure on the first decision.
    directory = tmp_path / 'simulation' / 'conveyor_supply'
    directory.mkdir(parents=True)
    first = directory / 'decision-00000001.json'
    first.write_text('{}')
    adapter.set_live_test_session_id('storage-failure', tmp_path)
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
        lease = {'reservation_id': 'accepted'}
        packets.append(dict(op='reservation', reservation=lease))
        adapter.tick(.01)
        assert adapter.reservation == lease
        assert adapter.packet['op'] == 'command'
        assert spawned == [1] and adapter.feeder.remaining == 1
        assert (outgoing[-1]['created_cycle'], outgoing[-1]['created_type_id']) == (1, 'ConveyorPart')
    finally:
        adapter.set_live_test_session_id(None)
        clear_supply_port(adapter.feeder)
    assert first.read_text() == '{}'
    summary = json.loads((directory / 'capture_summary.json').read_text())
    assert summary['write_failure_count'] == 1 and not summary['capture_complete']


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


def test_session_records_feed_decisions_at_transitions(tmp_path, monkeypatch):
    import json
    from dataclasses import asdict, replace
    from types import SimpleNamespace
    from unittest.mock import Mock
    from conveyor_pnp_cell import runtime
    from conveyor_pnp_cell.equipment_io import Commands, Sensors

    now = 100.
    monkeypatch.setattr(runtime.time, 'monotonic', lambda: now)
    packets = []
    def poll():
        result = packets.copy()
        packets.clear()
        return result
    ipc = SimpleNamespace(available=True, send=lambda _: None, poll_messages=poll, close=lambda: None)
    sensors = Sensors(path_clear=True, pick_zone_clear=True)
    equipment = SimpleNamespace(stream_id='stream', cycle=0, material_type_id='', drive=Mock(), step=Mock(),
                                observe=lambda permitted: replace(sensors, permitted=permitted),
                                spawn=Mock(return_value=True))
    adapter = runtime.ConveyorRuntime(dict(cell=dict(configured_supply=dict(material_id='A'),
        conveyor=dict(material_type_ids=['A'], pick_slot_id='pick', connection={}, equipment_permit_topic='permit',
                      teaching=dict(cycle=None)))), equipment, ipc)
    idle = Commands(0, 0, False, False, False, False, False, 0, 0, 0, 0)
    feed = replace(idle, phase=10, cycle=1, feed=True)
    def tick(command, permit_at, permitted=True):
        packets.append(dict(op='command', at=now, command=asdict(command), permitted=permitted, permit_at=permit_at))
        adapter.tick(.01)
    def records(root):
        files = (root/'simulation/conveyor_supply').glob('decision-*.json')
        return [json.loads(path.read_text()) for path in sorted(files)]
    try:
        adapter.set_live_test_session_id('session-a', tmp_path)
        adapter.tick(.01)
        tick(feed, 0., False)
        tick(idle, 0., False)
        tick(idle, now)
        now += .1
        tick(idle, now)
        adapter.feeder.configure('A', 2)
        adapter.feeder.set_automatic(True)
        tick(feed, now - 2)
        tick(feed, now, False)
        sensors.occupied = True
        tick(feed, now)
        sensors.occupied = False
        tick(feed, now)
        adapter.set_live_test_session_id('session-b', tmp_path/'next')
        tick(feed, now)
        now += 2.
        adapter.tick(.01)
        packets.append(dict(op='offline'))
        adapter.tick(.01)
        adapter.tick(.01)
    finally:
        adapter.close()
    rows = records(tmp_path)
    assert len(rows) == 8
    assert all(row['live_test_session_id'] == 'session-a' and row['stream_id'] == 'stream' for row in rows)
    assert rows[0]['command_status'] == 'not_received' and rows[0]['plc'] is None
    assert rows[1]['plc'] == dict(phase=10, cycle=1, committed=0, feed=True)
    assert not rows[1]['synchronized'] and rows[2]['synchronized']
    assert rows[2]['permit_status'] == 'not_received' and rows[2]['permit_value'] is None
    assert rows[3]['permit_status'] == 'fresh' and rows[3]['automatic'] is False
    assert rows[4]['permit_status'] == 'expired' and rows[4]['permit_age_s'] == 2
    assert rows[5]['permit_status'] == 'fresh' and rows[5]['permit_value'] is False
    assert rows[6]['occupied'] and rows[6]['blockers'] == []
    assert 'not_next_material' in rows[7]['blockers'] and rows[7]['remaining'] == 1
    equipment.spawn.assert_called_once_with('A', 1)
    current, expired, offline = records(tmp_path/'next')
    assert current['live_test_session_id'] == 'session-b' and current['last_generated'] == 1
    assert expired['command_status'] == 'expired' and expired['command_age_s'] == 2
    assert offline['command_status'] == 'offline' and offline['permit_status'] == 'unknown'
    assert offline['plc'] is None and offline['path_clear'] is None


def test_paused_scene_waits_for_plc_origin_without_authorizing_feed():
    from dataclasses import asdict
    from time import monotonic
    from types import SimpleNamespace
    from conveyor_pnp_cell.runtime import ConveyorRuntime
    from conveyor_pnp_cell.equipment_io import Commands, Sensors
    from isaac_runtime.cell_supply import clear_supply_port

    now = monotonic()
    cmd = Commands(1, 0, False, False, False, False, False, 1, 4, 0, 2)
    packet = dict(op='command', at=now, command=asdict(cmd), permitted=True, permit_at=now)
    sent = []
    ipc = SimpleNamespace(available=True, send=sent.append, poll_messages=lambda: [packet])
    equipment = SimpleNamespace(
        stream_id='new', cycle=0, path=None, material_type_id='', step=lambda *_: None,
        observe=lambda permitted: Sensors(permitted=permitted, stopped=True, path_clear=True,
                                          pick_zone_clear=True, discharge_zone_clear=True))
    configuration = dict(cell=dict(configured_supply=dict(material_id='ConveyorPart'), conveyor=dict(
        material_type_ids=['ConveyorPart'], pick_slot_id='cell/belt/pick', connection={},
        equipment_permit_topic='permit', teaching=dict(cycle=None))))
    adapter = ConveyorRuntime(configuration, equipment, ipc)
    try:
        adapter.tick(.01)
        assert not adapter.synchronized and not sent[-1]['valid']
        assert sent[-1]['created_cycle'] == 0 and not adapter.feeder.automatic
    finally:
        clear_supply_port(adapter.feeder)


def test_device_commands_do_not_require_process_records():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from conveyor_pnp_cell.scene import Equipment

    eq = Equipment.__new__(Equipment)
    eq.drive, eq.stop_position, eq.stage = Mock(), Mock(), Mock()
    eq.stop_position.Get.return_value = [0., 0., 0.]
    eq.layout = dict(speed_m_s=.5, stopper_retracted_m=[0., 0., 0.],
                     stopper_travel_m=.1, stopper_speed_m_s=.1)
    eq.cycle, eq.path, eq.spawned_ns = 7, '/part', 10
    sensors = SimpleNamespace(drive_available=True, occupied=False)
    cmd = SimpleNamespace(run=True, extend=True, discharge=False)
    eq.step(cmd, sensors, 1., enabled=True)
    eq.drive.assert_called_once_with(.5)
    eq.stop_position.Set.assert_called_once_with([0., -.1, 0.])
    eq.stage.RemovePrim.assert_not_called()
    sensors.occupied = cmd.discharge = True
    eq.step(cmd, sensors, 1., enabled=False)
    eq.drive.assert_called_with(0.)
    eq.stage.RemovePrim.assert_not_called()
    eq.step(cmd, sensors, 1., enabled=True)
    eq.stage.RemovePrim.assert_called_once_with('/part')
    assert eq.path is None and eq.last_discharge[0] == 7
