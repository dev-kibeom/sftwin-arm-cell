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
