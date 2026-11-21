"""System-Python owner of Modbus, equipment permits and configured-supply ROS services."""
import socket
import sys
import time
from dataclasses import asdict
from pathlib import Path
from threading import Event, Lock, Thread
from types import SimpleNamespace
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_cell_interfaces.msg import ConfiguredSupplyState
from arm_cell_interfaces.srv import ConfiguredSupply
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import rclpy
from std_msgs.msg import Bool
from tape_roll_cell.configured_supply_sidecar import ConfiguredSupplyClient
from conveyor_pnp_cell.configured_supply import ConveyorPickSupply
from conveyor_pnp_cell.equipment_io import ConveyorIO, Sensors
from conveyor_pnp_cell.runtime import MAX_AGE
from conveyor_pnp_cell.supply import bind_confirmed_pick


class Exchange:
    def __init__(self, node, ipc, config):
        self.node, self.ipc = node, ipc
        link = config['connection']
        self.client = ModbusTcpClient(link['host'], port=link['port'], timeout=link['timeout_seconds'])
        self.io = ConveyorIO(self.client, link['unit_id'])
        self.pick_configured = config['pick_configured']
        self.supply = ConveyorPickSupply(uuid4().hex, **config['supply'])
        self.permit_at = self.valid_at = 0.
        self.at_pick = self.permitted = False
        self.sent = {}
        self.publisher = node.create_publisher(ConfiguredSupplyState, '/cell/configured_supply/state', 10)
        node.create_service(ConfiguredSupply, '/cell/configured_supply', self.reserve)
        node.create_subscription(Bool, config['permit_topic'], self.permit, 1)
        self.lock = Lock()
        self.shutdown = Event()
        self.interval = link['poll_seconds']
        self.worker = Thread(target=self.run, name='conveyor_modbus')
        node.create_timer(.1, self.publish_supply)
        self.worker.start()

    def run(self):
        try:
            while self.ipc.available and not self.shutdown.is_set():
                self.poll()
                self.shutdown.wait(self.interval)
        finally:
            self.ipc.available = False
            self.client.close()

    def close(self):
        self.shutdown.set()
        self.worker.join()

    def publish_supply(self):
        with self.lock:
            reservable = self.supply.available and not self.supply.reservation
            state = self.supply.state(valid=self.valid() and (not reservable or self.at_pick))
        self.publisher.publish(state)

    def permit(self, message):
        with self.lock:
            self.permit_at = time.monotonic()
            self.permitted = message.data

    def valid(self):
        return time.monotonic() - self.valid_at < MAX_AGE

    def reserve(self, request, response):
        with self.lock:
            return self.supply.handle(request, response, valid=self.valid() and self.at_pick)

    def sample(self, packet, command):
        sensors = Sensors(**packet['sensors'])
        observed = packet['observed']
        valid = packet['valid']
        at_pick = (valid and observed == command.cycle and sensors.pick and sensors.stopped
                   and sensors.extended and not sensors.occupied)

        with self.lock:
            bound = 0
            material_type_id = packet['material_type_id']
            if at_pick and self.pick_configured:
                bound = bind_confirmed_pick(command, observed, material_type_id, self.supply.expose_arrival)
            self.at_pick = (at_pick and command.confirmed and self.pick_configured
                            and self.supply.available and observed == self.supply.cycle
                            and material_type_id == self.supply.material_type_id)
            self.valid_at = packet['acquired'] if valid else 0.
        self.io.publish(valid=valid, feeder=SimpleNamespace(**packet['feeder']),
                        sensors=sensors, bound_cycle=bound)

    def poll(self):
        now = time.monotonic()
        self.sent = {at: cmd for at, cmd in self.sent.items() if now-at < MAX_AGE}
        samples = self.ipc.poll_messages()
        try:
            for packet in reversed(samples):
                old = self.sent.pop(packet.get('at'), None)
                if old is not None and now-packet['acquired'] < MAX_AGE and packet.get('op') == 'sample':
                    # The acquisition belongs to one PLC command and one physical frame.
                    self.sample(packet, old)
                    break
            cmd = self.io.commands()
            at = time.monotonic()
            self.sent[at] = cmd
            with self.lock:
                permitted, permit_at = self.permitted, self.permit_at
                supply = self.supply
                reservation = dict(supply_id=supply.supply_id, material_id=supply.material_id,
                                   slot_id=supply.slot, supply_epoch=supply.epoch,
                                   material_type_id=supply.material_type_id,
                                   material_instance_id=supply.material_instance_id,
                                   reservation_id=supply.reservation) if supply.reservation and self.valid() else None
            self.ipc.send(dict(op='command', at=at, command=asdict(cmd), permitted=permitted,
                               permit_at=permit_at, reservation=reservation))
        except (OSError, ValueError, ModbusException) as error:
            if self.sent or self.valid_at:
                self.node.get_logger().warning(f'Conveyor link unavailable: {error}')
            self.client.close()
            self.sent.clear()
            with self.lock:
                self.valid_at = 0.
            self.ipc.send(dict(op='offline'))


def serve(fd):
    ipc = ConfiguredSupplyClient(socket.socket(fileno=fd))
    rclpy.init()
    node = rclpy.create_node('conveyor_equipment')
    exchange = None
    try:
        ipc.send(dict(op='ready', ready=True))
        while rclpy.ok() and ipc.available:
            if exchange is None:
                for packet in ipc.poll_messages():
                    if packet.get('op') == 'configure':
                        exchange = Exchange(node, ipc, packet)
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if exchange:
            exchange.close()
        ipc.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    serve(int(sys.argv[1]))
