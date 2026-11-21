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
                                   registers=[0] * 29 + [signature])

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
    assert reads == [(0, 30, 1), 'closed']


@pytest.mark.parametrize('isaac_conveyor', [True, False])
def test_adapter_uses_conveyor_process_params_and_session_endpoint(monkeypatch, tmp_path, isaac_conveyor):
    index = ModuleType('ament_index_python.packages')
    index.get_package_prefix = lambda _: str(tmp_path)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', index)
    launched = []
    monkeypatch.setattr(conveyor_plc.os, 'execv', lambda program, args: launched.append(args))
    isaac = {'material_supply_factory': 'conveyor_pnp_cell.runtime:create_adapter'} if isaac_conveyor else {}
    conveyor_plc.start_adapter(dict(cell=dict(isaac=isaac, conveyor=dict(
        equipment_permit_topic='/test/equipment_permit',
        plc_profile='package://arm_cell_plc_adapter/config/conveyor_pnp_codesys.yaml',
        connection=dict(host='127.0.0.1', port=11502, unit_id=3, poll_seconds=.1, timeout_seconds=.8),
        receipt_file='/tmp/selected stream.sqlite'))))
    args, = launched
    assert args[1:4] == ['-m', 'arm_cell_motion.launch_output',
                         str(tmp_path / 'lib/arm_cell_plc_adapter/plc_task_adapter')]
    assert [Path(args[i+1]).name for i, arg in enumerate(args) if arg == '--params-file'] == [
        'conveyor_pnp_codesys.yaml', 'conveyor_process.yaml']
    assert ('simulation_equipment_permit_topic:=/test/equipment_permit' in args) == isaac_conveyor
    assert 'port:=11502' in args and 'unit_id:=3' in args
    assert 'receipt_file:=/tmp/selected stream.sqlite' in args


def test_only_service_start_and_stop_use_opted_in_host_rule(tmp_path, monkeypatch):
    import plc_control
    policy = tmp_path / 'policy'
    monkeypatch.setattr(plc_control, 'service_policy_path', lambda: policy)
    assert plc_control.service_command('start') == ['systemctl', 'start', '--no-ask-password', 'codesyscontrol.service']
    policy.touch()
    for verb in ('start', 'stop'):
        assert plc_control.service_command(verb) == [
            '/usr/bin/sudo', '-n', '-k', '--', '/usr/bin/systemctl', verb, '--no-ask-password', 'codesyscontrol.service']
    with pytest.raises(ValueError):
        plc_control.service_command('restart')


@pytest.mark.parametrize('phase', [0, 10, 30, 39, 51])
@pytest.mark.parametrize('case', ['reset', 'old_program', 'no_ack', 'lost_ack', 'bad_origin'])
def test_initialize_after_simulation_shutdown(tmp_path, monkeypatch, case, phase):
    base = tmp_path / 'plc.sqlite'
    base.write_bytes(b'previous test')
    (tmp_path / 'plc.test-1.sqlite').touch()
    times = iter(n / 10 for n in range(1000))
    monkeypatch.setattr(conveyor_plc.time, 'monotonic', lambda: next(times))
    monkeypatch.setattr(conveyor_plc.time, 'sleep', lambda _: None)
    writes = []

    def reply(code, **kw):
        return SimpleNamespace(isError=lambda: False, function_code=code, **kw)

    class PLC:
        epoch = 0
        def __init__(self, *args, **kwargs):
            pass

        def read_input_registers(self, address, count, unit):
            capability = 0x1a18 if case == 'old_program' else 0x1a19
            words = [0] * 29 + [0xc051, capability, self.epoch]
            if not self.epoch:
                words[1], words[17], words[18] = 2, 1, phase
            elif case == 'bad_origin':
                words[27] = 4
            return reply(4, registers=words[address:address + count])

        def write_registers(self, address, words, unit):
            assert (address, words) == (26, [2])
            writes.append(2)
            if case != 'no_ack':
                self.epoch = 2
            if case == 'lost_ack':
                raise OSError('reply lost after write')
            return reply(16, address=address, count=1)

        def close(self):
            pass

    monkeypatch.setitem(sys.modules, 'pymodbus.client.sync', SimpleNamespace(ModbusTcpClient=PLC))
    monkeypatch.setitem(sys.modules, 'pymodbus.exceptions', SimpleNamespace(ModbusException=OSError))
    layout = dict(receipt_file=str(base), connection=dict(
        host='127.0.0.1', port=1502, timeout_seconds=.5, unit_id=1, poll_seconds=.1))
    if case in ('reset', 'lost_ack'):
        assert conveyor_plc.initialize_test(layout) == str(tmp_path / 'plc.test-2.sqlite')
        assert writes == [2]
    else:
        with pytest.raises((ValueError, OSError)):
            conveyor_plc.initialize_test(layout)
        assert bool(writes) == (case != 'old_program')
    assert base.read_bytes() == b'previous test'
