"""Exercise the offline entry point against closed owner databases and saved diagnostics."""
from pathlib import Path
from types import SimpleNamespace
import json
import sqlite3
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'infra/isaac_sim/scripts'), str(ROOT / 'ros2_ws/src/plc_adapter')]
from conveyor_pnp_cell.production_ledger import ProductionLedger
from arm_cell_plc_adapter.receipts import Receipts, State

CLI = ROOT / 'tools/analyze_conveyor_cycles.py'


def analyze(directory, count=1):
    result = subprocess.run([
        sys.executable, str(CLI), str(directory / 'production.sqlite'), '--stream', 'cell',
        '--count', str(count), '--diagnostic', str(directory), '--json',
    ], check=True, capture_output=True, text=True)
    return json.loads(result.stdout)


def save_run(directory, count=1):
    gripper = directory / 'gripper'
    gripper.mkdir()
    mission_dir = directory / 'orchestration'
    mission_dir.mkdir()
    ledger = ProductionLedger(directory / 'production.sqlite', 'cell')
    receipts = Receipts(directory / 'plc.sqlite')
    ledger.bind_receipt_source(directory / 'plc.sqlite')
    receipts.observe_idle()
    with (mission_dir / 'configured-supply-1.jsonl').open('w') as events:
        for number in range(1, count + 1):
            reservation = f'lease-{number}'
            instance = f'cell/{number}'
            snapshot = dict(stream_id='cell', plc_number=number, type_id='part', epoch='epoch',
                            slot_id='pick', available=True, reservation_id='')
            ledger.record_created(number, 'part')
            for phase in (10, 20, 22):
                ledger.record_plc(SimpleNamespace(cycle=number, phase=phase, result_state=0,
                    reason=0, placement=0, committed=number - 1))
            ledger.save_supply(snapshot)
            ledger.save_supply(dict(snapshot, reservation_id=reservation))
            ledger.record_plc(SimpleNamespace(cycle=number, phase=30, result_state=0, reason=0,
                                              placement=0, committed=number - 1))
            receipt = receipts.claim(number)
            task = number.to_bytes(16, 'big')
            receipts.update(number, State.SUCCEEDED, 7, 2, SimpleNamespace(
                task_goal_id=SimpleNamespace(uuid=task), material_instance_id=instance,
                reservation_id=reservation, task_status=4, result_code=0, failure_stage='',
                diagnostic_detail=''))
            identity = dict(reservation_id=reservation, material_instance_id=instance,
                            material_type_id='part', supply_epoch='epoch', slot_id='pick')
            for event, facts in [('reservation_confirmed', {}), ('task_accepted', {'task_goal_uuid': task.hex()}),
                                 ('task_outcome', dict(task_status=4, task_result_code=0, material_placed=True))]:
                events.write(json.dumps(dict(component='orchestration', event=event, **identity,
                    execute_cycle_id=receipt.goal_uuid.hex(), observed_monotonic_ns=100, **facts)) + '\n')
            for phase, holding, sec in [('capture', 'held', 1), ('release', 'released', 4)]:
                record = dict(record_type='physical_grasp', execution_id=task.hex(), phase=phase,
                              holding_state=holding, observed_monotonic_ns=sec * 10**9,
                              coverage={'producer_instance': 'tool-a'}, reference=dict(
                                  **identity, strategy='surface_suction', motion_goal_uuid=task.hex()))
                (gripper / f'suction-{number}-{phase}.json').write_text(json.dumps(record))
            ledger.save_supply(dict(snapshot, available=False), released=True)
            for phase in (50, 51, 0):
                ledger.record_plc(SimpleNamespace(cycle=number, phase=phase, result_state=4,
                                                  reason=7, placement=2, committed=number))
            ledger.record_discharge(number, 10**9, (number + 1) * 10**9)
    receipts.close()
    ledger.close()


def test_complete_contiguous_window_and_local_durations(tmp_path):
    save_run(tmp_path, 30)
    report = analyze(tmp_path, 30)
    assert report['verdict'] == 'PASS'
    assert report['counts'] == {'PASS': 30, 'FAIL': 0, 'UNKNOWN': 0}
    assert report['durations_s']['cycle'] == dict(n=30, mean=15.5, median=15.5, maximum=30)
    assert report['durations_s']['held_to_release']['mean'] == 3


def test_missing_material_and_missing_tool_evidence(tmp_path):
    save_run(tmp_path)
    (tmp_path / 'gripper/suction-1-capture.json').unlink()
    report = analyze(tmp_path, 2)
    assert report['counts']['UNKNOWN'] == 2
    assert report['durations_s']['held_to_release']['n'] == 0
    assert report['success_rate'] is None


@pytest.mark.parametrize('mutation', ['identity', 'task_failure', 'duplicate_task'])
def test_confirmed_defects_survive_plc_commit(tmp_path, mutation):
    save_run(tmp_path)
    with sqlite3.connect(tmp_path / 'plc.sqlite') as db:
        if mutation == 'task_failure':
            db.execute("UPDATE receipt SET state=6,reason=3 WHERE command_id=1")
            db.execute("UPDATE configured_task SET task_outcome=?", (json.dumps(dict(
                status=6, result_code=5, failure_stage='RETREAT', diagnostic_detail='blocked')),))
        elif mutation == 'identity':
            db.execute("UPDATE configured_task SET material_instance_id='wrong/1'")
        else:
            db.execute("INSERT INTO receipt VALUES (2,randomblob(16),4,7,2)")
            db.execute("INSERT INTO configured_task SELECT 2,material_instance_id,reservation_id,"
                       "lower(hex(randomblob(16))),task_outcome FROM configured_task WHERE command_id=1")
    report = analyze(tmp_path)
    assert report['verdict'] == 'FAIL'
    assert report['cycles'][0]['stages']['commit']


def test_receipt_replay_and_recovered_delay_are_not_another_task_or_commit(tmp_path):
    save_run(tmp_path)
    owner = ProductionLedger(tmp_path / 'production.sqlite', 'cell')
    owner.record_discharge(1, 10**9, 2 * 10**9)
    lease = dict(stream_id='cell', plc_number=1, epoch='epoch', reservation_id='lease-1', slot_id='pick')
    owner.record_reconnect('SIDECAR_RECONNECTED', lease)
    owner.record_reconnect('RESERVATION_RECONCILED', lease, result='retained', body='cell/1', plc_number=1)
    owner.close()
    capture = tmp_path / 'orchestration/configured-supply-1.jsonl'
    with capture.open('a') as sink:
        for transition in ('warning', 'error', 'recovered'):
            sink.write(json.dumps(dict(component='orchestration', event='supply_observation_delay',
                transition=transition, material_instance_id='cell/1', reservation_id='lease-1')) + '\n')
    report = analyze(tmp_path)
    assert report['verdict'] == 'PASS'
    assert [event['transition'] for event in report['recovery'] if 'transition' in event] == [
        'warning', 'error', 'recovered']
    assert any(r.get('facts', {}).get('result') == 'retained' for r in report['recovery'])


def test_duplicate_commit_register_transition_and_retained_levels(tmp_path):
    save_run(tmp_path)
    owner = ProductionLedger(tmp_path / 'production.sqlite', 'cell')
    for previous, register in [(1, 0), (0, 1)]:
        owner.record_plc(SimpleNamespace(cycle=1, phase=0, result_state=4, reason=7, placement=2,
                                         committed=register), previous)
    owner.close()
    assert any(f['stage'] == 'commit' for f in analyze(tmp_path)['failures'])


def test_no_commit_evidence_after_release_is_unknown(tmp_path):
    save_run(tmp_path)
    with sqlite3.connect(tmp_path / 'production.sqlite') as db:
        db.execute("DELETE FROM history_events WHERE event_type='DISCHARGED' OR "
                   "(source='plc' AND json_extract(details,'$.committed')=1)")
        db.execute("UPDATE plc_cycles SET committed=0,phase=40")
    report = analyze(tmp_path)
    assert report['verdict'] == 'UNKNOWN'
    assert report['cycles'][0]['stages']['place'] and not report['cycles'][0]['stages']['commit']


def test_distinct_pre_execution_busy_allows_next_attempt(tmp_path):
    save_run(tmp_path)
    with sqlite3.connect(tmp_path / 'plc.sqlite') as db:
        db.execute("UPDATE receipt SET command_id=2 WHERE command_id=1")
        db.execute("UPDATE configured_task SET command_id=2 WHERE command_id=1")
        db.execute("INSERT INTO receipt VALUES (1,randomblob(16),6,9,1)")
        db.execute("INSERT INTO configured_task SELECT 1,material_instance_id,reservation_id,"
                   "lower(hex(randomblob(16))),? FROM configured_task WHERE command_id=2", (json.dumps(dict(
                       status=6, result_code=10, failure_stage='configured_supply:execution_admission')),))
    assert analyze(tmp_path)['verdict'] == 'PASS'


def test_older_records_keep_explicit_task_failure_when_receipts_unavailable(tmp_path):
    save_run(tmp_path)
    (tmp_path / 'plc.sqlite').unlink()
    path = tmp_path / 'orchestration/configured-supply-1.jsonl'
    data = list(map(json.loads, path.read_text().splitlines()))
    data[-1].update(task_status=5, task_result_code=5, material_placed=False,
                    failure_stage='approach', diagnostic_detail='canceled')
    path.write_text('\n'.join(map(json.dumps, data)) + '\n')
    report = analyze(tmp_path)
    assert any(f['stage'] == 'approach' for f in report['failures'])
    assert report['input_issues'] and not (tmp_path / 'plc.sqlite').exists()


def test_legacy_clock_and_truncated_capture(tmp_path):
    save_run(tmp_path)
    path = tmp_path / 'gripper/suction-1-release.json'
    record = json.loads(path.read_text())
    record['coverage']['producer_instance'] = 'another-writer'
    path.write_text(json.dumps(record))
    result = analyze(tmp_path)
    assert result['durations_s']['held_to_release']['n'] == 0
    path.write_text('{')
    assert analyze(tmp_path)['verdict'] == 'UNKNOWN'


def test_discharge_while_waiting_for_next_admission_is_incomplete(tmp_path):
    save_run(tmp_path)
    with sqlite3.connect(tmp_path / 'production.sqlite') as db:
        db.execute("DELETE FROM history_events WHERE source='plc' AND json_extract(details,'$.phase')=0")
        db.execute("UPDATE plc_cycles SET phase=51")
    cycle, = analyze(tmp_path)['cycles']
    assert cycle['verdict'] == 'UNKNOWN' and cycle['stages']['discharge']
    assert not cycle['stages']['plc_reusable']


def test_poll_can_miss_idle_before_observing_next_allocation(tmp_path):
    save_run(tmp_path, 2)
    with sqlite3.connect(tmp_path / 'production.sqlite') as db:
        db.execute("DELETE FROM history_events WHERE source='plc' AND instance_id='cell/1' "
                   "AND json_extract(details,'$.phase')=0")
        db.execute('UPDATE plc_cycles SET phase=51 WHERE plc_number=1')
    cycle, = analyze(tmp_path)['cycles']
    assert cycle['verdict'] == 'PASS'
    assert cycle['next_allocation']['instance_id'] == 'cell/2'
    assert cycle['durations_s']['p1_to_plc_reusable'] is None


def test_unresolved_receipt_is_preserved_and_input_is_never_created(tmp_path):
    save_run(tmp_path)
    with sqlite3.connect(tmp_path / 'plc.sqlite') as db:
        db.execute("UPDATE receipt SET state=3,reason=0,placement=0 WHERE command_id=1")
        db.execute('UPDATE configured_task SET task_outcome=NULL')
    result = analyze(tmp_path)
    assert result['verdict'] == 'UNKNOWN'
    with sqlite3.connect(tmp_path / 'plc.sqlite') as db:
        assert db.execute('SELECT state FROM receipt WHERE command_id=1').fetchone() == (3,)
    empty = tmp_path / 'empty'
    empty.mkdir()
    assert analyze(empty)['verdict'] == 'UNKNOWN'
    assert not list(empty.iterdir())


def test_retained_result_with_missing_reservation_cannot_pass(tmp_path):
    save_run(tmp_path)
    with sqlite3.connect(tmp_path / 'production.sqlite') as db:
        db.execute("DELETE FROM reservations")
    assert analyze(tmp_path)['verdict'] == 'UNKNOWN'


def test_old_receipt_with_only_mission_uuid_uses_explicit_diagnostic_links(tmp_path):
    save_run(tmp_path)
    with sqlite3.connect(tmp_path / 'plc.sqlite') as db:
        db.execute('DROP TABLE configured_task')
    cycle, = analyze(tmp_path)['cycles']
    assert cycle['verdict'] == 'PASS'
    assert cycle['attempts'][0]['link_source'] == 'mission diagnostics'
    assert cycle['attempts'][0]['command_id'] == 1


def test_discharge_report_distinguishes_direct_indirect_inferred_and_missing_evidence(tmp_path):
    save_run(tmp_path, 4)
    with sqlite3.connect(tmp_path / 'production.sqlite') as db:
        db.execute("DELETE FROM history_events WHERE event_type='DISCHARGED' AND instance_id!='cell/1'")
        db.execute("DELETE FROM history_events WHERE source='plc' AND json_extract(details,'$.phase')=0 "
                   "AND instance_id IN ('cell/3','cell/4')")
        db.execute("UPDATE plc_cycles SET phase=51 WHERE plc_number IN (3,4)")

    cycles = analyze(tmp_path, 4)['cycles']
    assert [c['discharge_evidence'] for c in cycles] == [
        'ISAAC_DISCHARGED', 'PLC_REUSABLE', 'NEXT_ALLOCATION', 'UNKNOWN']
    assert [c['verdict'] for c in cycles] == ['PASS', 'PASS', 'PASS', 'UNKNOWN']
    assert [c['stages']['discharge'] for c in cycles] == [True, True, True, False]
    assert [c['durations_s']['cycle'] for c in cycles] == [1, None, None, None]

    output = subprocess.check_output([sys.executable, str(CLI), str(tmp_path / 'production.sqlite'),
                                      '--stream', 'cell', '--count', '4', '--diagnostic', str(tmp_path)], text=True)
    for label, number in [('Direct (Isaac removal)', 1), ('Indirect (PLC reusable)', 2),
                          ('Inferred (next allocation)', 3), ('UNKNOWN', 4)]:
        assert f'{label}: {number}' in output
