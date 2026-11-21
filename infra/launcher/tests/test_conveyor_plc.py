"""Exercise the launcher's read-only Modbus probe and adapter handoff without a PLC."""
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import conveyor_plc


@pytest.mark.parametrize('signature', [0, 0xc051])
def test_probe_requires_the_conveyor_command_block(monkeypatch, signature):
    reads = []

    class Client:
        def __init__(self, host, port, timeout):
            assert (host, port, timeout) == ('example.invalid', 1502, .9)

        def read_input_registers(self, address, count, unit):
            reads.append((address, count, unit))
            return SimpleNamespace(isError=lambda: False, function_code=4,
                                   registers=[0] * 13 + [signature])

        def close(self):
            reads.append('closed')

    monkeypatch.setitem(sys.modules, 'pymodbus.client.sync', SimpleNamespace(ModbusTcpClient=Client))
    monkeypatch.setitem(sys.modules, 'pymodbus.exceptions', SimpleNamespace(ModbusException=OSError))
    link = dict(host='example.invalid', port=1502, timeout_seconds=.9, unit_id=1, poll_seconds=.1)
    if signature:
        conveyor_plc.probe(link)
    else:
        with pytest.raises(ValueError, match='Conveyor PnP PLC'):
            conveyor_plc.probe(link)
    assert reads == [(16, 14, 1), 'closed']


def test_adapter_uses_conveyor_process_params_and_session_endpoint(monkeypatch, tmp_path):
    index = ModuleType('ament_index_python.packages')
    index.get_package_prefix = lambda _: str(tmp_path)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', index)
    launched = []
    monkeypatch.setattr(conveyor_plc.os, 'execv', lambda program, args: launched.append(args))
    conveyor_plc.start_adapter(dict(cell=dict(conveyor=dict(
        plc_profile='package://arm_cell_plc_adapter/config/conveyor_pnp_codesys.yaml',
        connection=dict(host='127.0.0.1', port=11502, unit_id=3, poll_seconds=.1, timeout_seconds=.8),
        receipt_file='/tmp/selected stream.sqlite'))))
    args, = launched
    assert args[1] == str(tmp_path / 'lib/arm_cell_plc_adapter/plc_task_adapter')
    assert [Path(args[i+1]).name for i, arg in enumerate(args) if arg == '--params-file'] == [
        'conveyor_pnp_codesys.yaml', 'conveyor_process.yaml']
    assert 'port:=11502' in args and 'unit_id:=3' in args
    assert 'receipt_file:=/tmp/selected stream.sqlite' in args
