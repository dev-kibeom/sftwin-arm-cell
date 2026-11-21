from unittest.mock import Mock

import pytest
from rclpy.task import Future

from arm_cell_plc_adapter.node import PlcTaskAdapter
from arm_cell_plc_adapter.receipts import Receipts, State


@pytest.mark.parametrize('pending', ['acceptance', 'result'])
@pytest.mark.parametrize('outcome', ['cancelled', 'exception', 'empty'])
def test_unconfirmed_action_keeps_id_and_uuid(tmp_path, pending, outcome):
    """Inject local Future outcomes independently of the Modbus transport."""
    adapter = PlcTaskAdapter.__new__(PlcTaskAdapter)
    adapter.exchange = Mock()
    adapter.get_logger = Mock()
    adapter.mission = Mock()
    adapter.mission.server_is_ready.return_value = True
    adapter.acceptance = adapter.result = None
    adapter.connected = True
    adapter.simulation_permit = None
    adapter.receipts = Receipts(tmp_path / 'plc.sqlite')
    try:
        adapter.receipts.observe_idle()
        claimed = adapter.receipts.claim(1)
        future = Future()
        if outcome == 'cancelled':
            future.cancel()
        elif outcome == 'exception':
            future.set_exception(ValueError('failed response'))
        else:
            future.set_result(None)
        setattr(adapter, pending, future)
        adapter.poll()
        receipt = adapter.receipts.latest()
        assert receipt.command_id == 1
        assert receipt.goal_uuid == claimed.goal_uuid
        assert receipt.state == State.UNCERTAIN
        assert adapter.receipts.claim(2) is None
        assert adapter.acceptance is None and adapter.result is None
        adapter.mission.send_goal_async.assert_not_called()
        adapter.exchange.assert_called_once()
    finally:
        adapter.receipts.close()


def test_expired_admission_is_distinct_from_observed_false_even_at_rollover(monkeypatch):
    from std_msgs.msg import Bool
    from arm_cell_plc_adapter import mission_admission

    adapter = PlcTaskAdapter.__new__(PlcTaskAdapter)
    adapter.conveyor_test_stream = False
    adapter.plc = Mock(admission_offset=6)
    adapter.plc.read_command.return_value = 0
    adapter.receipts = Mock()
    adapter.admission = mission_admission.MissionAdmission()
    adapter.exchange()
    adapter.plc.write_admission.assert_called_with(0, False)
    adapter.admission.sequence = 65535
    monkeypatch.setattr(mission_admission.time, 'monotonic', lambda: 10.)
    adapter.admission.observe(Bool(data=False))
    adapter.exchange()
    adapter.plc.write_admission.assert_called_with(1, False)
    monkeypatch.setattr(mission_admission.time, 'monotonic', lambda: 11.)
    adapter.exchange()
    adapter.plc.write_admission.assert_called_with(0, False)
    adapter.admission.observe(Bool(data=True))
    adapter.exchange()
    adapter.plc.write_admission.assert_called_with(2, True)


def test_new_conveyor_test_can_claim_command_one_without_replaying_old_goal(tmp_path):
    path = tmp_path / 'plc.sqlite'

    def open_adapter(epoch):
        adapter = PlcTaskAdapter.__new__(PlcTaskAdapter)
        adapter.plc = Mock()
        adapter.plc.read_conveyor_command.return_value = (0, epoch)
        adapter.mission = Mock()
        adapter.admission = None
        adapter.submit = Mock()
        adapter.conveyor_test_stream = True
        adapter.receipts = adapter.receipt_epoch = None
        adapter.initialize_attempt = None
        adapter.receipt_base = path
        adapter.exchange()
        return adapter

    old = open_adapter(0)
    try:
        old.plc.read_conveyor_command.return_value = (1, 0)
        old.exchange()
        old_goal = old.receipts.latest().goal_uuid
        old.receipts.update(1, State.SUCCEEDED, placement=2)
        old.plc.read_conveyor_command.return_value = (1, 1)
        with pytest.raises(OSError, match='new robot session'):
            old.exchange()
        old.submit.assert_called_once()
        from arm_cell_plc_adapter.conveyor_stream import reserve_test_stream
        attempt = tmp_path / 'initialize'
        attempt.mkdir()
        assert reserve_test_stream(path, 0, attempt / 'stream') == 1
        assert reserve_test_stream(path, 1, attempt / 'stream') == 1
        new = open_adapter(1)
        try:
            new.initialize_attempt = attempt
            new.plc.read_conveyor_command.return_value = (0, 0)
            with pytest.raises(OSError, match='selected PLC stream'):
                new.exchange()
            new.plc.read_conveyor_command.return_value = (0, 1)
            new.plc.write_receipt.side_effect = OSError('lost acknowledgment')
            with pytest.raises(OSError, match='lost acknowledgment'):
                new.exchange()
            assert not (attempt / 'adapter-ready').exists()
            new.plc.write_receipt.side_effect = None
            new.exchange()
            assert (attempt / 'adapter-ready').read_text() == '1'
            new.plc.read_conveyor_command.return_value = (1, 1)
            new.exchange()
            new.submit.assert_called_once()
            assert new.receipts.latest().goal_uuid != old_goal
            assert old.receipts.latest().state == State.SUCCEEDED
        finally:
            new.receipts.close()
    finally:
        old.receipts.close()
