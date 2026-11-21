"""The UI configures stock. Only the identified equipment command may spawn."""
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from conveyor_pnp_cell.supply import ConveyorFeeder, bind_confirmed_pick
from isaac_runtime import cell_supply as port


def test_ui_settings_and_replayed_feed_do_not_spawn_twice():
    calls = []
    feeder = ConveyorFeeder('ConveyorPart', last_generated=0)
    feeder.bind_ui()
    try:
        assert port.request_supply('ConveyorPart', 2)['accepted']
        assert calls == []
        assert port.set_automatic(True)['accepted']
        feeder.feed(1, lambda material, cycle: calls.append((material, cycle)) or True)
        feeder.feed(1, lambda *_: calls.append('duplicate'))
        assert calls == [('ConveyorPart', 1)]
        assert feeder.remaining == 1 and feeder.last_generated == 1
        assert port.set_automatic(False)['accepted']
        feeder.feed(2, lambda *_: calls.append('disabled'))
        assert len(calls) == 1
    finally:
        port.clear_supply_port(feeder)


def test_uncertain_creation_requires_reconciliation():
    feeder = ConveyorFeeder('ConveyorPart', last_generated=8)
    feeder.configure('ConveyorPart', 2)
    feeder.set_automatic(True)
    calls = []
    def uncertain(material, cycle):
        calls.append(cycle)
        raise RuntimeError('creation outcome lost')
    with pytest.raises(RuntimeError):
        feeder.feed(9, uncertain)
    for command in (8, 9, 10):
        feeder.feed(command, uncertain)
    assert calls == [9] and feeder.last_generated == 8
    assert 'Awaiting creation confirmation for 9' in feeder.status()['detail']


def test_only_the_observed_and_exposed_pick_slot_can_be_acknowledged():
    command = SimpleNamespace(confirmed=True, cycle=12)
    calls = []

    def expose(cycle, material_type):
        calls.append((cycle, material_type))
        return True

    assert bind_confirmed_pick(command, 11, "A", expose) == 0 and calls == []
    assert bind_confirmed_pick(command, 12, "A", lambda *_: False) == 0
    assert bind_confirmed_pick(command, 12, "A", expose) == 12 and calls == [(12, 'A')]
    command.confirmed = False
    assert bind_confirmed_pick(command, 12, "A", expose) == 0 and calls == [(12, 'A')]


def test_successive_lots_keep_the_type_recorded_at_feed():
    feeder = ConveyorFeeder('ConveyorPart', last_generated=0, material_type_ids=['A', 'B'])
    receipts = []
    for cycle, part in enumerate(('A', 'B'), start=1):
        feeder.set_automatic(False)
        assert not feeder.configure('unknown', 1)['accepted']
        assert feeder.configure(part, 1)['accepted']
        feeder.set_automatic(True)
        feeder.feed(cycle, lambda material, number: receipts.append((number, material)) or True)
    assert receipts == [(1, 'A'), (2, 'B')]
