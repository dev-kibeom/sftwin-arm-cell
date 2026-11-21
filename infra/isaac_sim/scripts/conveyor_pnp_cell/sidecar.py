"""System-Python owner of Modbus, equipment permits and configured-supply ROS services."""
import socket
import sqlite3
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path
from threading import Event, Lock, Thread
from types import SimpleNamespace
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from arm_cell_interfaces.msg import ConfiguredSupplyState
from arm_cell_interfaces.srv import ConfiguredSupply, RecordConveyorInspection, QueryConveyorHistory
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool
from tape_roll_cell.configured_supply_sidecar import ConfiguredSupplyClient
from conveyor_pnp_cell.configured_supply import ConveyorPickSupply
from conveyor_pnp_cell.equipment_io import ConveyorIO, Sensors
from conveyor_pnp_cell.runtime import MAX_AGE
from conveyor_pnp_cell.supply import bind_confirmed_pick, material_instance_id
from conveyor_pnp_cell.production_ledger import ProductionLedger


class Exchange:
    def __init__(self, node, ipc, config):
        self.node, self.ipc = node, ipc
        link = config['connection']
        self.client = ModbusTcpClient(link['host'], port=link['port'], timeout=link['timeout_seconds'])
        self.io = ConveyorIO(self.client, link['unit_id'])
        self.pick_configured = config['pick_configured']
        self.supply = ConveyorPickSupply(uuid4().hex, **config['supply'])
        self.history_path = config['history_db_path']
        self.receipt_file = config.get('receipt_file')
        self.resume_stream = config.get('resume_stream', False)
        self.ledger = None
        self.restored_unverified = True
        self.identity_conflict = False
        self._last_birth = self._last_plc = None
        self._open_history()
        self.permit_at = self.valid_at = 0.
        self.published_observation = None
        self.at_pick = self.permitted = False
        self.sent = {}
        self.publisher = node.create_publisher(ConfiguredSupplyState, '/cell/configured_supply/state', 10)
        node.create_service(ConfiguredSupply, '/cell/configured_supply', self.reserve)
        node.create_service(RecordConveyorInspection, '/cell/conveyor/record_inspection', self.record_inspection)
        node.create_service(QueryConveyorHistory, '/cell/conveyor/query_history', self.query_history)
        node.create_subscription(Bool, config['permit_topic'], self.permit, 1)
        self.lock = Lock()
        self.shutdown = Event()
        self.interval = link['poll_seconds']
        self.worker = Thread(target=self.run, name='conveyor_modbus')
        node.create_timer(.1, self.publish_supply)
        self.worker.start()

    def _open_history(self):
        self.next_history_retry = time.monotonic() + 1.
        ledger = None
        try:
            ledger = ProductionLedger(self.history_path, self.supply.stream_id)
            previous = ledger.supply_snapshot()
            if previous:
                self.supply.restore_snapshot(previous)
            elif self.resume_stream:
                raise ValueError('Missing snapshot for the existing equipment stream')
            else:
                ledger.save_supply(self.supply.snapshot())
        except (OSError, ValueError, sqlite3.Error) as error:
            if ledger is not None:
                ledger.close()
            self.node.get_logger().warning(f'Conveyor supply history unavailable: {error}',
                                           throttle_duration_sec=5.)
            return
        self.ledger = ledger
        self.restored_unverified = bool(previous and previous['available'])
        if self.receipt_file:
            try:
                ledger.bind_receipt_source(self.receipt_file)
            except (OSError, sqlite3.Error) as error:
                self.node.get_logger().warning(f'Cannot index PLC receipt source: {error}')

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
        if self.ledger is not None:
            self.ledger.close()

    def publish_supply(self):
        with self.lock:
            reservable = self.supply.available and not self.supply.reservation
            pending = not self.valid() or self.restored_unverified
            state = self.supply.state(valid=not pending and not self.identity_conflict
                                      and (not reservable or self.at_pick))
            state.observation_pending = pending and not self.identity_conflict and self.ledger is not None
            # Repeating an old acquisition must not refresh DDS consumer admission.
            observation = (self.valid_at, state.reservation_id, state.valid, state.observation_pending)
            if state.valid and observation == self.published_observation:
                return
            self.published_observation = observation
        self.publisher.publish(state)

    def permit(self, message):
        with self.lock:
            self.permit_at = time.monotonic()
            self.permitted = message.data

    def valid(self):
        return time.monotonic() - self.valid_at < .5

    def reserve(self, request, response):
        with self.lock:
            if self.ledger is None:
                response.diagnostic_detail = 'Conveyor supply history unavailable'
                return response
            if self.restored_unverified and request.command == request.RESERVE:
                response.diagnostic_detail = 'Recovered reservation requires physical reconciliation'
                return response
            if (request.command == request.COMPLETE and request.material_placed and
                    request.reservation_id != self.supply.reservation and
                    request.supply_id == self.supply.supply_id and request.supply_epoch == self.supply.epoch):
                try:
                    response.success = self.ledger.release_recorded(request)
                except sqlite3.Error as error:
                    response.diagnostic_detail = f'Production history unavailable: {error}'
                response.supply_epoch = self.supply.epoch
                return response
            if request.command == request.COMPLETE and self.identity_conflict:
                response.diagnostic_detail = 'PLC cycle or physical material no longer matches reservation'
                return response
            before = self.supply.snapshot()
            response = self.supply.handle(request, response, valid=self.valid() and self.at_pick)
            after = self.supply.snapshot()
            if response.success and after != before:
                try:
                    self.ledger.save_supply(after, released=bool(request.material_placed))
                    if not after['available']:
                        self.restored_unverified = False
                except (ValueError, sqlite3.Error) as error:
                    self.supply.restore_snapshot(before)
                    return ConfiguredSupply.Response(
                        supply_epoch=self.supply.epoch, diagnostic_detail=f'Production history unavailable: {error}')
            return response

    def record_inspection(self, request, response):
        if self.ledger is None:
            response.diagnostic_detail = 'Conveyor history unavailable'
            return response
        try:
            self.ledger.inspect(
                instance_id=request.material_instance_id,
                inspection_id=request.inspection_id, station_id=request.station_id,
                verdict=request.verdict, evidence_ref=request.evidence_ref)
            response.accepted = True
        except (ValueError, sqlite3.Error) as error:
            response.diagnostic_detail = str(error)
        return response

    def query_history(self, request, response):
        if self.ledger is None:
            response.diagnostic_detail = 'Conveyor history unavailable'
            return response
        try:
            report = (self.ledger.history(request.material_instance_id)
                      if request.material_instance_id else self.ledger.summary())
            if report is None:
                response.diagnostic_detail = 'Unknown material instance'
            else:
                if request.material_instance_id:
                    report['robot_execution'] = self.execution_history(report['material'])
                response.success = True
                response.json_result = json.dumps(report, sort_keys=True)
        except sqlite3.Error as error:
            response.diagnostic_detail = str(error)
        return response

    def execution_history(self, material):
        source = self.receipt_file if material['stream_id'] == self.supply.stream_id else None
        if not source:
            source = self.ledger.receipt_source(material['stream_id'])
        if not source:
            return dict(status='source_not_configured', attempts=[])
        try:
            from arm_cell_plc_adapter.receipts import read_material_attempts
            attempts = read_material_attempts(source, material['instance_id'])
        except (ImportError, OSError, ValueError, sqlite3.Error) as error:
            return dict(status='source_unavailable', diagnostic_detail=str(error), attempts=[])
        return dict(status='linked' if attempts else 'no_link_observed', attempts=attempts)

    def sample(self, packet, command):
        sensors = Sensors(**packet['sensors'])
        observed = packet['observed']
        valid = packet['valid']
        at_pick = (valid and observed == command.cycle and sensors.pick and sensors.stopped
                   and sensors.extended and not sensors.occupied
                   and packet['body_instance_id'] == material_instance_id(self.supply.stream_id, observed)
                   and packet['material_type_id'] == packet['created_type_id'])

        with self.lock:
            if self.ledger is None and time.monotonic() >= self.next_history_retry:
                self._open_history()
            material_type_id = packet['material_type_id']
            bound = 0
            if at_pick and self.pick_configured and self.ledger is not None:
                before = self.supply.snapshot()
                bound = bind_confirmed_pick(command, observed, material_type_id, self.supply.expose_arrival)
                after = self.supply.snapshot()
                if before != after:
                    try:
                        self.ledger.save_supply(after)
                    except (ValueError, sqlite3.Error) as error:
                        self.supply.restore_snapshot(before)
                        bound = 0
                        self.node.get_logger().warning(f'Pick assignment not persisted: {error}',
                                                       throttle_duration_sec=5.)
            self.at_pick = (at_pick and command.confirmed and self.pick_configured
                            and self.supply.available and observed == self.supply.cycle
                            and material_type_id == self.supply.material_type_id)
            if self.at_pick:
                self.restored_unverified = False
            if self.supply.reservation and packet.get('identity_observed', valid):
                self.identity_conflict = self.identity_conflict or (
                    command.cycle != self.supply.cycle or command.phase not in (22, 23, 30, 40)
                    or packet['body_instance_id'] != self.supply.material_instance_id
                    or packet['created_type_id'] != self.supply.material_type_id)
                if valid and not self.identity_conflict and command.phase in (30, 40):
                    self.restored_unverified = False
            elif not self.supply.reservation:
                self.identity_conflict = False
            self.valid_at = packet['acquired'] if valid else 0.
        try:
            self.io.publish(valid=valid, feeder=SimpleNamespace(**packet['feeder']),
                            sensors=sensors, bound_cycle=bound)
        finally:
            if valid and self.ledger is not None:
                self._record_observations(packet, command)

    def _record_observations(self, packet, command):
        birth = (packet['created_cycle'], packet['created_type_id'])
        if birth != self._last_birth:
            try:
                self.ledger.record_created(*birth)
                self._last_birth = birth
            except (ValueError, sqlite3.Error) as error:
                self.node.get_logger().warning(f'Material observation not persisted: {error}',
                                               throttle_duration_sec=5.)
        plc = (command.cycle, command.phase, command.committed,
               command.result_state, command.reason, command.placement)
        if plc != self._last_plc:
            try:
                self.ledger.record_plc(command)
                self._last_plc = plc
            except (ValueError, sqlite3.Error) as error:
                self.node.get_logger().warning(f'PLC observation not persisted: {error}',
                                               throttle_duration_sec=5.)

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
                                   reservation_id=supply.reservation) if (
                    supply.reservation and self.valid() and not self.restored_unverified and not self.identity_conflict) else None
                pending = (not self.valid() or self.restored_unverified) and not self.identity_conflict and self.ledger is not None
            self.ipc.send(dict(op='command', at=at, command=asdict(cmd), permitted=permitted,
                               permit_at=permit_at, reservation=reservation, observation_pending=pending))
        except (OSError, ValueError, ModbusException) as error:
            if self.sent or self.valid_at:
                self.node.get_logger().warning(f'Conveyor equipment unavailable: {error}', throttle_duration_sec=5.)
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
            try:
                rclpy.spin_once(node, timeout_sec=.05)
            except (ExternalShutdownException, RuntimeError):
                if rclpy.ok():
                    raise
    finally:
        if exchange:
            exchange.close()
        ipc.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    serve(int(sys.argv[1]))
