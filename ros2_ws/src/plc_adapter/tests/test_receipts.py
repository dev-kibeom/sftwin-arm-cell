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
