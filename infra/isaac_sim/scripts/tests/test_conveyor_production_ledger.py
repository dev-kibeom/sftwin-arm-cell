"""Pure SQLite tests: production identity, durable recovery and independent quality evidence."""
from dataclasses import dataclass
from pathlib import Path
import sys
import sqlite3
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from conveyor_pnp_cell.production_ledger import ProductionLedger


def snapshot(cycle=1, *, available=True, reservation='', epoch='epoch-a', type_id='A'):
    return dict(stream_id='stream-a', plc_number=cycle, type_id=type_id,
                slot_id='cell/belt/pick', epoch=epoch, available=available,
                reservation_id=reservation)


@dataclass
class Plc:
    cycle: int = 1
    phase: int = 30
    result_state: int = 0
    reason: int = 0
    placement: int = 0
    committed: int = 0


def test_transactional_reservation_commit_inspection_and_restart(tmp_path):
    path = tmp_path/'cell.sqlite'
    writer = ProductionLedger(path, 'stream-a')
    writer.save_supply(snapshot())
    writer.save_supply(snapshot(reservation='lease-1'))
    writer.record_plc(Plc(phase=40, result_state=4, placement=2, committed=1))
    writer.save_supply(snapshot(available=False), released=True)
    assert writer.inspect(instance_id='stream-a/1', inspection_id='q1',
                          station_id='gauge-1', verdict='PASS', evidence_ref='measurement-1')
    assert not writer.inspect(instance_id='stream-a/1', inspection_id='q1',
                              station_id='gauge-1', verdict='PASS', evidence_ref='measurement-1')
    assert writer.summary()['plc_commits'] == 1
    writer.close()
    reopened = ProductionLedger(path, 'stream-a')
    assert reopened.supply_snapshot()['plc_number'] == 1
    part = reopened.history('stream-a/1')
    assert part['material']['type_id'] == 'A'
    assert part['plc']['committed'] == 1
    assert part['reservations'][0]['status'] == 'RELEASED'
    assert part['inspections'][0]['verdict'] == 'PASS'
    assert reopened.summary()['unresolved_reservations'] == 0
    reopened.close()


def test_rollback_on_conflicting_type_or_inspection(tmp_path):
    ledger = ProductionLedger(tmp_path/'history.sqlite', 'stream-a')
    ledger.save_supply(snapshot())
    with pytest.raises(ValueError):
        ledger.save_supply(snapshot(type_id='B'))
    assert ledger.supply_snapshot()['type_id'] == 'A'
    with pytest.raises(ValueError):
        ledger.inspect(instance_id='stream-a/1', inspection_id='q1',
                       station_id='inspection', verdict='PASS', evidence_ref='')
    ledger.inspect(instance_id='stream-a/1', inspection_id='q1',
                   station_id='inspection', verdict='FAIL', evidence_ref='evidence-1')
    with pytest.raises(ValueError):
        ledger.inspect(instance_id='stream-a/1', inspection_id='q1',
                       station_id='inspection', verdict='PASS', evidence_ref='evidence-2')
    assert len(ledger.history('stream-a/1')['inspections']) == 1
    ledger.close()


def test_multiple_streams_and_unresolved_reservation_survive_restart(tmp_path):
    path=tmp_path/'ledger.sqlite'
    first = ProductionLedger(path, 'stream-a')
    first.save_supply(snapshot(reservation='lease-1'))
    first.close()
    fresh = ProductionLedger(path, 'stream-b')
    assert fresh.supply_snapshot() is None
    fresh.save_supply(dict(snapshot(), stream_id='stream-b'))
    assert fresh.summary()['unresolved_reservations'] == 1
    assert fresh.history('stream-a/1')['reservations'][0]['status'] == 'ACTIVE'
    assert fresh.history('stream-b/1')['material']['stream_id'] == 'stream-b'
    fresh.close()

def test_physical_birth_precedes_pick_and_plc_result_is_not_fabricated(tmp_path):
    ledger = ProductionLedger(tmp_path/'born.sqlite', 'stream-a')
    ledger.record_created(7, 'B')
    ledger.record_created(7, 'B')
    assert ledger.history('stream-a/7')['plc'] is None
    assert ledger.history('stream-a/7')['reservations'] == []
    assert ledger.summary()['plc_commits'] == 0
    with pytest.raises(ValueError):
        ledger.record_created(7, 'A')
    ledger.record_plc(Plc(cycle=7, phase=50, result_state=4, placement=2, committed=7))
    assert ledger.summary()['plc_commits'] == 1
    ledger.record_plc(Plc(cycle=8, phase=10, result_state=0, placement=0, committed=7))
    assert ledger.summary()['plc_commits'] == 1
    ledger.close()
