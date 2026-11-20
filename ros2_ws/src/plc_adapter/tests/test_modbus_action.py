"""Loopback PLC registers -> deployed adapter -> configured Mission -> test Robot Task."""
from contextlib import contextmanager, ExitStack
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from arm_cell_interfaces.action import ExecuteConfiguredGrasp
from arm_cell_interfaces.msg import ConfiguredSupplyState, SafetyState
from arm_cell_interfaces.srv import ConfiguredSupply
from domain_coordinator import domain_id
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext, ModbusSlaveContext
from pymodbus.server.sync import ModbusTcpServer
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arm_cell_plc_adapter.receipts import State


def wait_for(condition, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(.02)
    assert condition()


@contextmanager
def process(package, executable, *args):
    binary = Path(get_package_prefix(package)) / 'lib' / package / executable
    proc = subprocess.Popen([str(binary), *args])
    try:
        yield proc
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


@contextmanager
def plc_server(bank, port=0):
    context = ModbusServerContext(slaves={1: bank}, single=False)
    server = ModbusTcpServer(context, address=('127.0.0.1', port), allow_reuse_address=True)
    thread = threading.Thread(target=server.serve_forever, args=(.02,))
    thread.start()
    try:
        yield server.server_address[1]
    finally:
        server.shutdown()
        for handler in list(server.threads):
            try:
                handler.request.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        server.server_close()
        thread.join()


class RobotFixture(Node):
    """Test-only supply, Safety observations and deliberately delayed Task terminal."""
    def __init__(self):
        super().__init__('plc_test_robot')
        group = ReentrantCallbackGroup()
        self.inventory = ConfiguredSupplyState(
            valid=True, supply_id='test_supply', material_id='test_part', supply_epoch='epoch',
            slot_ids=['first', 'second'], occupied=[True, True])
        self.safety = SafetyState(valid=True, required_inputs_fresh=True,
                                  motion_envelope_valid=True, max_velocity_scale=1.,
                                  max_acceleration_scale=1.)
        self.safety.motion_capability.value = 2
        self.safety_pub = self.create_publisher(SafetyState, '/safety/state', 10)
        self.supply_pub = self.create_publisher(ConfiguredSupplyState, '/cell/configured_supply/state', 10)
        self.create_timer(.04, self.publish)
        self.allow_reservation = threading.Event()
        self.create_service(ConfiguredSupply, '/cell/configured_supply', self.reserve,
                            callback_group=group)
        self.finish = threading.Event()
        self.goals = []
        self.task = ActionServer(self, ExecuteConfiguredGrasp,
                                 '/task_execution/execute_configured_grasp',
                                 self.execute, callback_group=group)

    def publish(self):
        self.safety_pub.publish(self.safety)
        self.supply_pub.publish(self.inventory)

    def reserve(self, req, res):
        supply = self.inventory
        if req.command == ConfiguredSupply.Request.RESERVE:
            assert self.allow_reservation.wait(2)
            slot = supply.slot_ids[supply.occupied.index(True)]
            supply.selected_slot_id, supply.reservation_id = slot, 'lease-' + slot
        else:
            assert req.reservation_id == supply.reservation_id
            if req.command == ConfiguredSupply.Request.COMPLETE:
                assert req.material_placed
                supply.occupied[supply.slot_ids.index(req.slot_id)] = False
            supply.selected_slot_id = supply.reservation_id = ''
        res.success = True
        res.supply_epoch, res.slot_id, res.reservation_id = (
            supply.supply_epoch, supply.selected_slot_id, supply.reservation_id)
        return res

    def execute(self, goal):
        self.goals.append(goal.request.slot_id)
        assert self.finish.wait(20)
        result = ExecuteConfiguredGrasp.Result(material_placed=True)
        if goal.request.slot_id == 'first':
            goal.succeed()
        else:
            result.result_code = 1
            result.failure_stage = 'RETREAT'
            goal.abort()
        return result


def test_plc_disconnect_and_restart_across_robot_results(tmp_path, monkeypatch):
    bank = ModbusSlaveContext(ir=ModbusSequentialDataBlock(0, [0] * 32),
                              hr=ModbusSequentialDataBlock(0, [0] * 32), zero_mode=True)
    def reply():
        return bank.getValues(3, 12, 6)

    with ExitStack() as stack:
        domain = stack.enter_context(domain_id())
        monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
        monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
        monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
        rclpy.init()
        stack.callback(rclpy.try_shutdown)
        robot = RobotFixture()
        stack.callback(robot.destroy_node)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(robot)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        stack.callback(thread.join)
        stack.callback(executor.shutdown)
        stack.callback(robot.finish.set)
        stack.enter_context(process('arm_cell_orchestration', 'configured_supply_mission.py',
                                    '--ros-args', '-p', 'supply_id:=test_supply',
                                    '-p', 'mission_target_id:=test_part'))
        plc = stack.enter_context(ExitStack())
        port = plc.enter_context(plc_server(bank))
        adapter = stack.enter_context(ExitStack())

        def start_adapter():
            adapter.enter_context(process('arm_cell_plc_adapter', 'plc_task_adapter',
                                  '--ros-args', '-p', 'host:=127.0.0.1', '-p', f'port:={port}',
                                  '-p', 'target_id:=test_part', '-p', 'poll_seconds:=0.04',
                                  '-p', 'request_input_offset:=7', '-p', 'reply_holding_offset:=12',
                                  '-p', 'request_fields:=[id_low, id_high]',
                                  '-p', f'receipt_file:={tmp_path / "plc.sqlite"}'))

        start_adapter()
        wait_for(lambda: reply()[5] > 1)
        wait_for(lambda: robot.supply_pub.get_subscription_count() > 0)
        bank.setValues(4, 7, [1, 0])
        wait_for(lambda: reply()[1:3] == [1, State.ACCEPTED])
        robot.allow_reservation.set()
        wait_for(lambda: reply()[1:3] == [1, State.RUNNING])
        heartbeat = reply()[5]
        wait_for(lambda: reply()[5] > heartbeat + 2)
        assert robot.goals == ['first']
        plc.close()
        robot.finish.set()
        wait_for(lambda: not robot.inventory.occupied[0])
        plc.enter_context(plc_server(bank, port))
        wait_for(lambda: reply()[1:5] == [1, State.SUCCEEDED, 7, 2])
        robot.finish.clear()

        bank.setValues(4, 7, [2, 0])
        wait_for(lambda: reply()[1:3] == [2, State.RUNNING])
        adapter.close()
        robot.finish.set()
        wait_for(lambda: not robot.inventory.reservation_id)
        start_adapter()
        wait_for(lambda: reply()[1:5] == [2, State.ABORTED, 3, 2])
        assert robot.inventory.occupied[1]
        bank.setValues(4, 7, [1, 0])
        wait_for(lambda: reply()[1:5] == [1, State.SUCCEEDED, 7, 2])
        # A new request rejected by Mission admission is also a durable receipt.
        robot.safety.valid = False
        time.sleep(.6)  # exceeds the configured Mission safety freshness window
        bank.setValues(4, 7, [3, 0])
        wait_for(lambda: reply()[1:5] == [3, State.REJECTED, 0, 0])
        assert robot.goals == ['first', 'second']
