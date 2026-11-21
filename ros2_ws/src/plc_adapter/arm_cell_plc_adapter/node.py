"""Modbus TCP client for a single configured Mission request stream."""
import math
from pathlib import Path
import sqlite3
import time

from action_msgs.msg import GoalStatus
from arm_cell_interfaces.action import ExecuteCycle
from pymodbus.exceptions import ModbusException
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from unique_identifier_msgs.msg import UUID

from .conveyor_stream import receipt_path
from .mission_admission import MissionAdmission
from .modbus import ModbusRegisters, REPLY_FIELDS, REQUEST_FIELDS
from .receipts import Receipts, State
from .simulation_permit import SimulationEquipmentPermit


RESULT_STATES = {
    GoalStatus.STATUS_SUCCEEDED: State.SUCCEEDED,
    GoalStatus.STATUS_CANCELED: State.CANCELED,
    GoalStatus.STATUS_ABORTED: State.ABORTED,
}


def completed_response(future, response_type):
    """Return None while pending; surface unconfirmed ROS responses to the receipt owner."""
    if future is None:
        return None
    if future.cancelled():
        raise RuntimeError('ROS response future was cancelled')
    if not future.done():
        return None
    error = future.exception()
    if error is not None:
        raise RuntimeError(f'ROS response future failed: {error}') from error
    response = future.result()
    if not isinstance(response, response_type):
        raise RuntimeError(f'Expected {response_type.__name__}, got {type(response).__name__}')
    return response


class PlcTaskAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('plc_task_adapter', **kwargs)

        def param(name, default):
            return self.declare_parameter(name, default).value

        period = param('poll_seconds', 0.1)
        self.target = param('target_id', '')
        action = param('mission_action', '/orchestration/execute_cycle')
        path = param('receipt_file', '')
        if not all((self.target, action, path)):
            raise ValueError('target_id, mission_action and receipt_file are required')
        if not math.isfinite(period) or period <= 0:
            raise ValueError('poll_seconds must be positive and finite')
        self.plc = ModbusRegisters(
            param('host', ''), param('port', 502), param('unit_id', 1),
            param('timeout_seconds', 0.25), param('request_input_offset', 0),
            param('reply_holding_offset', 0), param('request_fields', REQUEST_FIELDS),
            param('reply_fields', REPLY_FIELDS), param('admission_holding_offset', -1))
        self.admission = None
        if self.plc.admission_offset >= 0:
            self.admission = MissionAdmission()
            self.create_subscription(Bool, param('mission_admission_topic', '/orchestration/mission_admission_available'),
                                     self.admission.observe, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self.mission = ActionClient(self, ExecuteCycle, action)
        self.lookup = self.create_client(ExecuteCycle.Impl.GetResultService,
                                         action + '/_action/get_result')
        self.conveyor_test_stream = param('conveyor_test_stream', False)
        self.history_path = param('conveyor_history_file', '')
        attempt = param('initialize_attempt', '')
        self.initialize_attempt = Path(attempt) if attempt else None
        self.receipt_base = path
        self.receipt_epoch = None
        self.receipts = None if self.conveyor_test_stream else Receipts(path)
        self.acceptance = None
        self.goal_handle = None
        self.result = None
        self.next_lookup = 0.
        self.connected = None
        permit_topic = param('simulation_equipment_permit_topic', '')
        self.simulation_permit = SimulationEquipmentPermit(self, permit_topic) if permit_topic else None
        self.create_timer(period, self.poll)

    def submit(self, receipt):
        goal = ExecuteCycle.Goal(target_id=self.target, quantity=1)
        goal_uuid = UUID(uuid=list(receipt.goal_uuid))
        self.acceptance = self.mission.send_goal_async(
            goal, goal_uuid=goal_uuid, feedback_callback=self.on_feedback)

    def on_feedback(self, message):
        last = self.receipts.latest()
        if (last and last.goal_uuid == bytes(message.goal_id.uuid) and
                last.state in (State.SUBMITTED, State.ACCEPTED, State.RUNNING) and
                (last.state != State.RUNNING or message.feedback.configured_task.material_instance_id)):
            self.receipts.update(last.command_id, State.RUNNING, trace=message.feedback.configured_task)

    def observe_action(self):
        last = self.receipts.latest()
        if last is None:
            return
        handle = completed_response(self.acceptance, ClientGoalHandle)
        if handle is not None:
            self.acceptance = None
            self.goal_handle = handle
            if not self.goal_handle.accepted:
                self.receipts.update(last.command_id, State.REJECTED)
                self.goal_handle = None
                return
            if last.state == State.SUBMITTED:
                self.receipts.update(last.command_id, State.ACCEPTED)
            self.result = self.goal_handle.get_result_async()
        wrapped = completed_response(self.result, ExecuteCycle.Impl.GetResultService.Response)
        if wrapped is not None:
            self.result = None
            self.goal_handle = None
            state = RESULT_STATES.get(wrapped.status)
            if state is None:
                self.receipts.update(last.command_id, State.UNCERTAIN)
            else:
                payload = wrapped.result
                placement = (1 + int(payload.material_placed)) if payload.has_material_placed else 0
                self.receipts.update(last.command_id, state, payload.exit_reason.value, placement,
                                     trace=payload.configured_task)
            return
        if ((self.acceptance is not None or self.result is not None)
                and not self.mission.server_is_ready()):
            self.mark_uncertain(last.command_id, 'Action server disappeared')
            return
        if (last.state == State.UNCERTAIN and self.result is None and
                time.monotonic() >= self.next_lookup and self.lookup.service_is_ready()):
            request = ExecuteCycle.Impl.GetResultService.Request()
            request.goal_id.uuid = list(last.goal_uuid)
            self.next_lookup = time.monotonic() + 2.
            self.result = self.lookup.call_async(request)

    def exchange(self):
        if self.conveyor_test_stream:
            command_id, epoch = self.plc.read_conveyor_command()
            if self.initialize_attempt is not None:
                expected = int((self.initialize_attempt / 'stream').read_text())
                if epoch != expected or command_id != 0:
                    raise OSError('Waiting for the selected PLC stream origin')
            if self.receipts is None:
                self.receipts = Receipts(receipt_path(self.receipt_base, epoch))
                self.receipt_epoch = epoch
            elif epoch != self.receipt_epoch:
                raise OSError('PLC initialized; waiting for a new robot session')
        else:
            command_id = self.plc.read_command()
        if command_id == 0:
            self.receipts.observe_idle()
        elif self.mission.server_is_ready():
            claimed = self.receipts.claim(command_id)
            if claimed:
                try:
                    self.submit(claimed)
                except RuntimeError as error:
                    self.mark_uncertain(claimed.command_id, error)
        receipt = self.receipts.reply(command_id)
        if self.conveyor_test_stream:
            if self.initialize_attempt is not None and (receipt.command_id != 0 or receipt.state != State.IDLE):
                raise ValueError('Selected PLC stream contains a previous receipt')
            closed = 0
            if command_id == 0 and receipt.command_id:
                try:
                    if self.receipts.discharged_command(receipt, self.history_path):
                        closed = receipt.command_id
                except sqlite3.Error as error:
                    self.get_logger().warning(f'Cannot reconcile previous Conveyor cycle: {error}',
                                              throttle_duration_sec=5.)
                if not closed:
                    self.get_logger().warning(
                        f'PLC requests 0; cannot confirm discharge for previous robot request {receipt.command_id}',
                        throttle_duration_sec=5.)
            admission = self.admission.register_values() if self.admission is not None else (0, False)
            # One scan sees the heartbeat, receipt, admission and completed ID from one FC16.
            feedback = (admission[0], int(admission[1]), closed >> 16, closed & 0xffff, self.receipt_epoch)
            self.plc.write_receipt(receipt, conveyor_feedback=feedback)
            if self.initialize_attempt is not None:
                ready = self.initialize_attempt / 'adapter-ready'
                temporary = ready.with_suffix('.tmp')
                temporary.write_text(str(self.receipt_epoch))
                temporary.replace(ready)
                self.initialize_attempt = None
            return
        self.plc.write_receipt(receipt)
        if self.admission is not None:
            self.plc.write_admission(*self.admission.register_values())

    def mark_uncertain(self, command_id, error):
        # Drop local waits only. The robot can still complete this Goal UUID.
        for future in (self.acceptance, self.result):
            if future is not None and not future.done():
                future.cancel()
        self.acceptance = self.result = self.goal_handle = None
        self.receipts.update(command_id, State.UNCERTAIN)
        self.next_lookup = time.monotonic() + 2.
        self.get_logger().warning(f'Cannot confirm Mission for command {command_id}: {error}')

    def poll(self):
        try:
            if self.receipts is not None:
                self.observe_action()
        except RuntimeError as error:
            self.mark_uncertain(self.receipts.latest().command_id, error)
        try:
            self.exchange()
        except (ModbusException, OSError) as error:
            self.plc.close()
            if self.connected is not False:
                self.get_logger().warning(f'PLC exchange unavailable: {error}')
            self.connected = False
        else:
            if not self.connected:
                self.get_logger().info('PLC register exchange synchronized')
            self.connected = True
        if self.simulation_permit is not None:
            self.simulation_permit.observe_exchange(self.connected)

    def destroy_node(self):
        self.plc.close()
        if self.receipts is not None:
            self.receipts.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PlcTaskAdapter()
        executor = MultiThreadedExecutor(num_threads=2) if node.simulation_permit or node.admission else None
        try:
            rclpy.spin(node, executor=executor)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except (RuntimeError, TypeError) as error:
            # Humble can wrap SIGINT during Python message conversion in take().
            if node.context.ok() and not isinstance(error.__cause__, KeyboardInterrupt):
                raise
        finally:
            if executor is not None:
                executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.try_shutdown()
