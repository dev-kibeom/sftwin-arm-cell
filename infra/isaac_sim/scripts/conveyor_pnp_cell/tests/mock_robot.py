"""Commissioning only: robot I/O writer, simulated permit and real supply client."""
import argparse
from pathlib import Path
import time

from pymodbus.client.sync import ModbusTcpClient
import rclpy
from std_msgs.msg import Bool
from arm_cell_interfaces.msg import ConfiguredSupplyState
from arm_cell_interfaces.srv import ConfiguredSupply

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('exchange_dir', type=Path)
    parser.add_argument('--robot-port', type=int, required=True)
    args = parser.parse_args()
    plc = ModbusTcpClient('127.0.0.1', port=args.robot_port, timeout=.9)
    rclpy.init()
    node = rclpy.create_node('mock_conveyor_robot')
    service = node.create_client(ConfiguredSupply, '/cell/configured_supply')
    states = []
    node.create_subscription(ConfiguredSupplyState, '/cell/configured_supply/state',
                             lambda msg: states.__setitem__(slice(None), [msg]), 1)
    permit = node.create_publisher(Bool, '/cell/conveyor/equipment_permitted', 10)
    from threading import Event, Thread
    stop = Event()
    def publish_permit():
        while not stop.wait(.1) and rclpy.ok():
            permit.publish(Bool(data=True))
    thread = Thread(target=publish_permit)
    thread.start()
    sequence = completed = 0
    lease = None
    delayed_receipt = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            sequence = sequence % 65535 + 1
            ir = plc.read_input_registers(0, 30, unit=1)
            if ir.isError():
                raise RuntimeError(str(ir))
            w = ir.registers
            assert w[29] == 0xc051
            hr = plc.read_holding_registers(16, 10, unit=1).registers
            if w[18] == 10 and hr[7] == 1 and not hr[5] & 1:
                delayed_receipt = True
            task = w[0] << 16 | w[1]
            if task > completed and states and states[-1].valid:
                req = ConfiguredSupply.Request(supply_id='conveyor_pick', supply_epoch=states[-1].supply_epoch)
                if lease is None:
                    req.command = req.RESERVE
                elif hr[5] & 16:
                    req.command = req.COMPLETE
                    req.slot_id, req.reservation_id = lease.slot_id, lease.reservation_id
                    req.material_instance_id, req.material_type_id = lease.material_instance_id, lease.material_type_id
                    req.material_placed = True
                else:
                    req = None
                if req is not None:
                    future = service.call_async(req)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
                    result = future.result()
                    assert result and result.success, (result, w, hr, states[-1])
                    if req.command == req.RESERVE:
                        assert result.slot_id == 'conveyor_pnp_cell/CV-01/pick'
                        assert result.material_instance_id.rsplit('/', 1)[-1] == str(task)
                        assert hr[9] == task or w[18] == 30
                        lease = result
                        (args.exchange_dir/f'place-{task}').touch()
                    else:
                        completed, lease = task, None
            reply = [0, completed, 4 if completed else 0, 0, 2 if completed else 0, sequence, sequence, 1]
            assert not plc.write_registers(0, reply, unit=1).isError()
            if completed == 2 and w[18] == 0:
                assert delayed_receipt
                print('PASS: observed separate creation and infeed, two reserved and released slots', flush=True)
                return
            time.sleep(.025)
    finally:
        stop.set()
        thread.join()
        plc.close()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
