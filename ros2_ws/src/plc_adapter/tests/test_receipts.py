import sqlite3

import pytest

from arm_cell_plc_adapter.receipts import Receipts, State


def test_duplicate_and_restart_never_reclaim_a_goal(tmp_path):
    path = tmp_path / 'cell.sqlite'
    db = Receipts(path)
    assert db.claim(1) is None  # uncommissioned, PLC has an outstanding request
    db.observe_idle()
    first = db.claim(1)
    assert first.goal_uuid and first.state == State.SUBMITTED
    assert db.claim(1) is None
    assert db.claim(2) is None
    db.close()

    db = Receipts(path)
    assert db.latest().goal_uuid == first.goal_uuid
    assert db.latest().state == State.UNCERTAIN
    assert db.claim(2) is None
    db.update(1, State.ABORTED, reason=3, placement=2)
    assert db.claim(3) is None
    assert db.claim(2)
    assert db.claim(1) is None
    old = db.reply(1)
    assert (old.command_id, old.state, old.reason, old.placement) == (1, State.ABORTED, 3, 2)
    db.close()


def test_second_adapter_cannot_take_ownership_of_active_request(tmp_path):
    path = tmp_path / 'plc.sqlite'
    owner = Receipts(path)
    owner.observe_idle()
    owner.claim(1)
    with pytest.raises(sqlite3.OperationalError, match='locked'):
        Receipts(path)
    assert owner.latest().state == State.SUBMITTED
    owner.close()


def test_material_lookup_joins_each_attempt_to_its_original_receipt(tmp_path):
    from types import SimpleNamespace
    from arm_cell_plc_adapter.receipts import read_material_attempts

    path = tmp_path / 'receipt.sqlite'
    owner = Receipts(path)
    owner.observe_idle()
    first = owner.claim(1)
    trace = SimpleNamespace(material_instance_id='stage/7', reservation_id='lease1',
                            task_goal_id=SimpleNamespace(uuid=bytes(range(16))),
                            task_status=0, result_code=0, failure_stage='', diagnostic_detail='')
    owner.update(1, State.RUNNING, trace=trace)
    pending, = read_material_attempts(path, 'stage/7')
    assert pending['mission_goal_uuid'] == first.goal_uuid.hex() and pending['task_outcome'] is None
    trace.task_status, trace.result_code, trace.failure_stage = 6, 5, 'retreat'
    owner.update(1, State.ABORTED, reason=3, placement=2, trace=trace)
    second = owner.claim(2)
    trace.reservation_id, trace.task_status = 'lease2', 0
    trace.task_goal_id.uuid = bytes(16)
    owner.update(2, State.ACCEPTED, trace=trace)
    owner.close()
    failed, active = read_material_attempts(path, 'stage/7')
    assert (failed['command_id'], failed['reservation_id'], failed['task_goal_uuid']) == (1, 'lease1', bytes(range(16)).hex())
    assert failed['task_outcome'] == dict(status=6, result_code=5, material_placed=True,
                                          failure_stage='retreat', diagnostic_detail='')
    assert active['command_id'] == 2 and active['mission_goal_uuid'] == second.goal_uuid.hex()
    assert active['task_goal_uuid'] is None and active['task_outcome'] is None
    assert read_material_attempts(path, 'other-stage/7') == []
