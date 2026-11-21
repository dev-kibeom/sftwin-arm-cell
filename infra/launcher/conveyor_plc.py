"""Probe the selected Conveyor application or replace this process with its ROS adapter."""
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
            print(f'Conveyor PLC 응답 0xC051 · 단계 {command.phase} · 자재 번호 {command.cycle}', flush=True)
            return
        except (OSError, ModbusException) as error:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"{link['host']}:{link['port']} · {error}") from error
            time.sleep(link['poll_seconds'])
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
    args = [str(executable), '--ros-args', '--params-file', str(profile), '--params-file', str(process)]
    # Use the endpoint and receipt stream that Isaac received at session export.
    for key, value in dict(layout['connection'], receipt_file=layout['receipt_file']).items():
        args.extend(['-p', f'{key}:={value}'])
    os.execv(sys.executable, [sys.executable, *args])


if __name__ == '__main__':
    config = json.loads(Path(sys.argv[2]).read_text())['configuration']
    try:
        if sys.argv[1] == 'probe':
            probe(config['cell']['conveyor']['connection'])
        elif sys.argv[1] == 'adapter':
            start_adapter(config)
        else:
            raise ValueError('Unknown PLC launcher phase')
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        sys.exit(str(error))
