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
    from arm_cell_plc_adapter import node

    adapter = PlcTaskAdapter.__new__(PlcTaskAdapter)
    adapter.plc = Mock(admission_offset=6)
    adapter.plc.read_command.return_value = 0
    adapter.receipts = Mock()
    adapter.admission_sequence = 65535
    monkeypatch.setattr(node.time, 'monotonic', lambda: 10.)
    adapter.on_admission(Bool(data=False))
    adapter.exchange()
    adapter.plc.write_admission.assert_called_with(1, False)
    monkeypatch.setattr(node.time, 'monotonic', lambda: 11.)
    adapter.exchange()
    adapter.plc.write_admission.assert_called_with(0, False)
    adapter.on_admission(Bool(data=True))
    adapter.exchange()
    adapter.plc.write_admission.assert_called_with(2, True)
