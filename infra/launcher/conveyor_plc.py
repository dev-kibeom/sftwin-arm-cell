"""Probe, initialize or connect to the local Conveyor PLC."""
import ipaddress
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def probe(link):
    from pymodbus.client.sync import ModbusTcpClient
    from pymodbus.exceptions import ModbusException
    sys.path.insert(0, str(ROOT / 'infra/isaac_sim/scripts'))
    from conveyor_pnp_cell.equipment_io import ConveyorIO

    client = ModbusTcpClient(link['host'], port=link['port'], timeout=link['timeout_seconds'])
    io = ConveyorIO(client, link['unit_id'])
    deadline = time.monotonic() + 10
    while True:
        try:
            command = io.commands()
            version_note = ' · 이전 앱: 재시작 복구를 사용하려면 새 앱을 다운로드하세요' if command.requested is None else ''
            print(f'Conveyor PLC 응답 0xC051 · 단계 {command.phase} · 자재 번호 {command.cycle}{version_note}', flush=True)
            return
        except (OSError, ModbusException) as error:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"{link['host']}:{link['port']} · {error}") from error
            time.sleep(link['poll_seconds'])
        finally:
            client.close()


def initialize_test(layout, pending=None):
    """Request a new retained test and verify its idle PLC image."""
    from pymodbus.client.sync import ModbusTcpClient
    from pymodbus.exceptions import ModbusException
    sys.path.insert(0, str(ROOT / 'infra/isaac_sim/scripts'))
    sys.path.insert(0, str(ROOT / 'ros2_ws/src/plc_adapter'))
    from arm_cell_plc_adapter.conveyor_stream import receipt_path, reserve_test_stream
    from conveyor_pnp_cell.equipment_io import ConveyorIO

    link = layout['connection']
    if link['host'] != 'localhost' and not ipaddress.ip_address(link['host']).is_loopback:
        raise ValueError('Initialize requires the local simulation PLC')
    client = ModbusTcpClient(link['host'], port=link['port'], timeout=link['timeout_seconds'])
    io = ConveyorIO(client, link['unit_id'])
    try:
        epoch = io.test_epoch(require_offline_reset=True)
        next_epoch = reserve_test_stream(layout['receipt_file'], epoch, pending)
        # The last heartbeat may still be fresh when the old processes exit. Repeat
        # the same epoch, reading it back even if the write acknowledgement was lost.
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                current = io.test_epoch(require_offline_reset=True)
                if current == next_epoch:
                    io.verify_initialized(next_epoch)
                    break
                if current != epoch:
                    raise ValueError('PLC test number changed during Initialize')
                io.request_initialize(next_epoch)
            except (ModbusException, OSError):
                client.close()
            time.sleep(link['poll_seconds'])
        else:
            raise TimeoutError('Waiting for the previous writers to stop and the PLC to acknowledge Initialize')
        return receipt_path(layout['receipt_file'], next_epoch)
    except ModbusException as error:
        raise OSError(str(error)) from error
    finally:
        client.close()


def start_adapter(config):
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0, str(ROOT / 'ros2_ws/src/bringup'))
    from arm_cell_bringup.runtime import resolve

    layout = config['cell']['conveyor']
    package = 'arm_cell_plc_adapter'
    executable = Path(get_package_prefix(package)) / 'lib' / package / 'plc_task_adapter'
    profile = resolve(layout['plc_profile'], ROOT)
    process = resolve('package://arm_cell_plc_adapter/config/conveyor_process.yaml', ROOT)
    args = [sys.executable, '-m', 'arm_cell_motion.launch_output', str(executable),
            '--params-file', str(profile), '--params-file', str(process)]
    # Use the endpoint and receipt stream that Isaac received at session export.
    for key, value in dict(layout['connection'], receipt_file=layout['receipt_file']).items():
        args.extend(['-p', f'{key}:={value}'])
    if layout.get('history_db_path'):
        args.extend(['-p', f"conveyor_history_file:={layout['history_db_path']}"])
    attempt = os.environ.get('ARM_CELL_INITIALIZE_ATTEMPT')
    if attempt:
        args.extend(['-p', f'initialize_attempt:={attempt}'])
    factory = config['cell'].get('isaac', {}).get('material_supply_factory')
    if factory == 'conveyor_pnp_cell.runtime:create_adapter':
        args.extend(['-p', f"simulation_equipment_permit_topic:={layout['equipment_permit_topic']}"])
    os.execv(sys.executable, args)


if __name__ == '__main__':
    config = json.loads(Path(sys.argv[2]).read_text())['configuration']
    try:
        if sys.argv[1] == 'probe':
            probe(config['cell']['conveyor']['connection'])
        elif sys.argv[1] == 'initialize':
            attempt = Path(os.environ['ARM_CELL_INITIALIZE_ATTEMPT'])
            while True:
                try:
                    receipt_file = initialize_test(config['cell']['conveyor'], pending=attempt / 'stream')
                    break
                except OSError:
                    time.sleep(1)
            print(f'PLC 초기화 확인 · 새 영수증 경로 {receipt_file}', flush=True)
        elif sys.argv[1] == 'adapter':
            start_adapter(config)
        else:
            raise ValueError('Unknown PLC launcher phase')
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        sys.exit(str(error))
