#!/usr/bin/env python3
"""Configured-source Mission lifecycle. Vision acquisition is not part of this work source."""
import json
import logging
import os
from pathlib import Path
import queue
import threading
import time


class ConfiguredMissionDiagnostics:
    """Optional bounded decision records, independent of Mission execution."""
    def __init__(self, directory, enabled=False, logger=None):
        self.logger=logger or logging.getLogger(__name__)
        self.directory=Path(directory)
        self.enabled=enabled
        self.pending=queue.Queue(maxsize=64)
        self.stopped=threading.Event()
        self.dropped_count=0;self.failed_count=0;self.written_count=0
        self.worker=None
        if enabled:
            self.worker=threading.Thread(target=self._run,daemon=True,name='configured-mission-diagnostics')
            self.worker.start()

    def emit(self, event, **evidence):
        if not self.enabled:return
        record=dict(schema_version=1,component='orchestration',event=event,
                    observed_monotonic_ns=time.monotonic_ns(),**evidence)
        try:self.pending.put_nowait(record)
        except queue.Full:self.dropped_count+=1

    def _run(self):
        size=0;failure_cause=None;failure_count=0;drops_reported=False
        while not self.stopped.is_set() or not self.pending.empty():
            try:record=self.pending.get(timeout=.05)
            except queue.Empty:continue
            if self.dropped_count and not drops_reported:
                self._warning('Configured Mission diagnostic evidence is being dropped: queue overflow')
                drops_reported=True
            try:
                line=json.dumps(record,allow_nan=False)+'\n'
                encoded=len(line.encode('utf-8'))
                if encoded>65536 or size+encoded>16*1024*1024:
                    raise OSError('configured Mission diagnostic byte limit reached')
                self.directory.mkdir(parents=True,exist_ok=True)
                path=self.directory/f'configured-supply-{os.getpid()}.jsonl'
                with path.open('a',encoding='utf-8') as stream:stream.write(line)
                size+=encoded;self.written_count+=1
                if failure_cause is not None:
                    try:self.logger.info(f'Configured Mission diagnostic storage recovered / failed={failure_count}')
                    except Exception:pass
                    failure_cause=None;failure_count=0
            except Exception as error:
                self.failed_count+=1;failure_count+=1
                cause=(type(error).__name__,str(error))
                if cause!=failure_cause:
                    self._warning('Configured Mission diagnostic evidence unavailable; Mission results are unchanged / %s',error)
                    failure_cause=cause
            finally:self.pending.task_done()
        try:
            if self.written_count:
                summary=dict(schema_version=1,component='orchestration',
                    capture_complete=not(self.dropped_count or self.failed_count),
                    written_count=self.written_count,dropped_count=self.dropped_count,failed_count=self.failed_count)
                (self.directory/f'configured-supply-{os.getpid()}.summary.json').write_text(json.dumps(summary)+'\n')
        except Exception:
            self._warning('Configured Mission diagnostic coverage summary unavailable')
        if self.dropped_count:
            self._warning('Configured Mission diagnostic queue overflow / dropped=%d', self.dropped_count)

    def _warning(self,message,*args):
        try:self.logger.warning(message % args if args else message)
        except Exception:pass

    def close(self):
        self.stopped.set()
        if self.worker:self.worker.join(timeout=1.)
        complete=self.worker is None or not self.worker.is_alive()
        if not complete:self._warning('Configured Mission diagnostic shutdown incomplete; pending evidence may be lost')
        return complete


def mission_reason_name(reason, constants):
    return next((name for name in dir(constants) if name.startswith('MISSION_EXIT_')
                 and getattr(constants,name)==reason),'MISSION_EXIT_UNKNOWN')


def safety_permits(state, received_at, now):
    return bool(state is not None and now-received_at <= .5 and state.valid and
                state.required_inputs_fresh and state.motion_envelope_valid and
                state.motion_capability.value == 2 and state.max_velocity_scale > 0 and state.max_acceleration_scale > 0)


def motion_allows_admission(status, received_at, now):
    if status is None or now - received_at > 1. or status.execution_active or status.has_active_execution:
        return False
    return (status.execution_state == status.MOTION_STATE_IDLE or
            (status.execution_state == status.MOTION_STATE_STOPPED and status.backend_inactivity_confirmed))


def supply_permits(state, received_at, now, supply_id, material_id):
    return bool(state is not None and now-received_at <= .5 and state.valid and
                state.supply_id == supply_id and state.material_id == material_id and state.supply_epoch and
                len(state.slot_ids) == len(state.occupied))


def lease_matches(state, lease):
    return bool(state is not None and state.supply_epoch == lease.supply_epoch and
                state.selected_slot_id == lease.slot_id and state.reservation_id == lease.reservation_id and
                state.material_type_id == lease.material_type_id and
                state.material_instance_id == lease.material_instance_id and
                lease.slot_id in state.slot_ids and state.occupied[state.slot_ids.index(lease.slot_id)])


def cancel_abandoned_acceptance(future, abandoned):
    if abandoned.is_set() and future.done():
        late = future.result()
        if late is not None and late.accepted:
            late.cancel_goal_async()


def abandon_pending_acceptance(future, abandoned):
    abandoned.set()
    # Cover a result whose callback raced just before the event was set.
    cancel_abandoned_acceptance(future, abandoned)


class PendingTaskUncertain(RuntimeError):
    pass


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from arm_cell_interfaces.action import ExecuteCycle, ExecuteConfiguredGrasp
    from arm_cell_interfaces.srv import ConfiguredSupply
    from arm_cell_interfaces.msg import ConfiguredSupplyState, SafetyState, MissionExitReason, MissionPhase, MotionStatus

    from arm_cell_interfaces.msg import MotionTaskResultCode as TaskCode
    from std_msgs.msg import Bool
    from rclpy.qos import QoSProfile
    from rclpy.duration import Duration

    class Mission(Node):
        def __init__(self):
            super().__init__('orchestration_node')
            self.supply_id=self.declare_parameter('supply_id','tape_cartridge').value
            self.target=self.declare_parameter('mission_target_id','TapeRoll17').value
            self.safety=None;self.safety_at=0.;self.supply=None;self.supply_at=0.
            self.lock=threading.Lock();self.busy=False
            self.group=ReentrantCallbackGroup()
            self.create_subscription(SafetyState,'/safety/state',self.on_safety,10,callback_group=self.group)
            self.create_subscription(ConfiguredSupplyState,'/cell/configured_supply/state',self.on_supply,10,callback_group=self.group)
            self.cell=self.create_client(ConfiguredSupply,'/cell/configured_supply',callback_group=self.group)
            self.task=ActionClient(self,ExecuteConfiguredGrasp,'/task_execution/execute_configured_grasp',callback_group=self.group)
            self.server=None;self.startup_wait=None
            self.capture=ConfiguredMissionDiagnostics(
                self.declare_parameter('diagnostics.output_directory','').value,
                self.declare_parameter('diagnostics.capture_enabled',False).value,logger=self.get_logger())
            self.plc_admission = self.declare_parameter('plc_admission_enabled', False).value
            self.motion_sample = (None, 0.)
            if self.plc_admission:
                self.create_subscription(MotionStatus, '/motion/state', self.on_motion, 1)
                admission_topic = self.declare_parameter('mission_admission_topic',
                    '/orchestration/mission_admission_available').value
                self.admission_publisher = self.create_publisher(
                    Bool, admission_topic, QoSProfile(depth=1, lifespan=Duration(seconds=.5)))
                self.create_timer(.1, self.publish_admission)
            self.startup_timer=self.create_timer(.1,self.advertise_when_ready)

        def advertise_when_ready(self):
            task_ready=self.task.server_is_ready();cell_ready=self.cell.service_is_ready()
            reason=('Task and Cell supply unavailable' if not task_ready and not cell_ready else
                    'Task unavailable' if not task_ready else 'Cell supply unavailable' if not cell_ready else None)
            if reason!=self.startup_wait:
                self.log_event('debug','Configured Mission startup '+('waiting: '+reason if reason else 'wait ended: dependencies ready'))
                self.startup_wait=reason
            if self.server is None and reason is None:
                self.server=ActionServer(self,ExecuteCycle,'/orchestration/execute_cycle',execute_callback=self.execute,
                    goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
                self.startup_timer.cancel()

        def log_event(self, level, message):
            try:getattr(self.get_logger(),level)(message)
            except Exception:pass

        def capture_event(self, handle, event, **evidence):
            try:
                self.capture.emit(event,execute_cycle_id=bytes(handle.goal_id.uuid).hex(),
                    target_id=handle.request.target_id,requested_quantity=handle.request.quantity,**evidence)
            except Exception:
                self.log_event('warning','Configured Mission diagnostic submission unavailable; Mission results are unchanged')

        def interruption_evidence(self):
            now=time.monotonic()
            safety=self.safety;supply=self.supply
            return dict(safety_age_s=now-self.safety_at,supply_age_s=now-self.supply_at,
                safety=None if safety is None else dict(valid=safety.valid,inputs_fresh=safety.required_inputs_fresh,
                    envelope_valid=safety.motion_envelope_valid,motion_capability=safety.motion_capability.value,
                    max_velocity_scale=safety.max_velocity_scale,max_acceleration_scale=safety.max_acceleration_scale),
                supply=None if supply is None else dict(valid=supply.valid,supply_epoch=supply.supply_epoch,
                    selected_slot_id=supply.selected_slot_id,reservation_id=supply.reservation_id))

        def on_safety(self,state):self.safety=state;self.safety_at=time.monotonic()
        def on_supply(self,state):self.supply=state;self.supply_at=time.monotonic()

        def on_motion(self, status):
            self.motion_sample = (status, time.monotonic())

        def safety_allows_motion(self):
            return safety_permits(self.safety, self.safety_at, time.monotonic())

        def supply_is_current(self):
            return supply_permits(self.supply, self.supply_at, time.monotonic(), self.supply_id, self.target)

        def inputs_permit_execution(self):
            return self.safety_allows_motion() and self.supply_is_current()

        def available_for_job(self):
            return (not self.busy and self.inputs_permit_execution() and not self.supply.reservation_id
                    and (not self.plc_admission or motion_allows_admission(*self.motion_sample, time.monotonic()))
                    and self.task.server_is_ready() and self.cell.service_is_ready())

        def publish_admission(self):
            with self.lock:
                ready = self.server is not None and self.available_for_job()
                self.admission_publisher.publish(Bool(data=ready))

        def accept(self,goal):
            with self.lock:
                if (not self.available_for_job() or goal.target_id != self.target or
                        goal.quantity != 1 or any(goal.delivery_id.uuid)):
                    return GoalResponse.REJECT
                self.busy=True
            return GoalResponse.ACCEPT

        @staticmethod
        def wait(future,timeout):
            deadline=time.monotonic()+timeout
            while not future.done():
                if time.monotonic()>=deadline:raise TimeoutError('ROS operation result deadline exceeded')
                time.sleep(.01)
            if future.cancelled():
                raise RuntimeError('ROS operation response canceled')
            try:
                return future.result()
            except Exception as error:
                raise RuntimeError(f'ROS operation response unavailable: {error}') from error

        def command(self,command,epoch,lease=None,placed=False):
            request=ConfiguredSupply.Request();request.command=command;request.supply_id=self.supply_id
            request.supply_epoch=epoch
            request.material_placed=placed
            if lease is not None:
                request.slot_id=lease.slot_id
                request.reservation_id=lease.reservation_id
                request.material_type_id=lease.material_type_id
                request.material_instance_id=lease.material_instance_id
            for attempt in range(2):
                pending = self.cell.call_async(request)
                try:
                    return self.wait(pending, 3.5)
                except TimeoutError:
                    pending.cancel()
                    # Only resend cleanup for the exact lease after provider replacement.
                    if (attempt or command != request.ABANDON or not self.plc_admission or
                            not self.cell.wait_for_service(timeout_sec=1.)):
                        raise

        def feedback(self,handle,count,phase,detail,trace=None):
            message=ExecuteCycle.Feedback();message.phase.value=phase
            message.delivery_id=handle.request.delivery_id;message.quantity=handle.request.quantity
            message.batch_processed_count=count;message.diagnostic_detail=detail
            if trace is not None:
                message.configured_task = trace
            handle.publish_feedback(message)

        def execute(self,handle):
            result=ExecuteCycle.Result();sequence_completed=False;lease=None;child=None;terminal_known=True
            reason=MissionExitReason.MISSION_EXIT_MOTION_ERROR;detail='configured task did not complete'
            safety_preempted = False
            self.capture_event(handle,'mission_started')
            self.log_event('info',f'Configured Mission started / requested={handle.request.quantity}')
            try:
                epoch=self.supply.supply_epoch
                while not sequence_completed:
                    if handle.is_cancel_requested:
                        reason=MissionExitReason.MISSION_EXIT_CANCELED;detail='Canceled before slot reservation';break
                    if not self.inputs_permit_execution() or self.supply.supply_epoch != epoch:
                        reason=MissionExitReason.MISSION_EXIT_SAFETY_PREEMPTED;detail='Safety or configured supply became unavailable';break
                    reserved=self.command(ConfiguredSupply.Request.RESERVE,epoch)
                    if reserved.supply_epoch != epoch:
                        raise RuntimeError('Cell session changed during reservation')
                    if reserved.success and reserved.exhausted:
                        reason=MissionExitReason.MISSION_EXIT_QUANTITY_SHORTFALL;detail='Configured supply exhausted before requested quantity';lease=None;break
                    if not reserved.success:
                        raise RuntimeError('Cell reservation rejected: '+reserved.diagnostic_detail)
                    lease=reserved
                    trace = result.configured_task
                    trace.material_instance_id = lease.material_instance_id
                    trace.reservation_id = lease.reservation_id
                    self.feedback(handle, 0, MissionPhase.MISSION_PHASE_PICKING, 'Slot reserved', trace)
                    deadline=time.monotonic()+1.
                    while not (self.inputs_permit_execution() and lease_matches(self.supply,lease)):
                        if time.monotonic()>deadline or self.supply.supply_epoch != epoch or handle.is_cancel_requested:
                            raise RuntimeError('Cell reservation state not confirmed')
                        time.sleep(.01)
                    self.capture_event(handle,'reservation_confirmed',supply_epoch=lease.supply_epoch,
                        reservation_id=lease.reservation_id,slot_id=lease.slot_id,
                        material_type_id=lease.material_type_id,material_instance_id=lease.material_instance_id)
                    goal=ExecuteConfiguredGrasp.Goal();goal.supply_id=self.supply_id;goal.slot_id=lease.slot_id
                    goal.supply_epoch=lease.supply_epoch;goal.reservation_id=lease.reservation_id
                    goal.material_type_id=lease.material_type_id;goal.material_instance_id=lease.material_instance_id
                    abandoned=threading.Event()
                    terminal_known=False
                    sent=self.task.send_goal_async(goal)
                    def accepted_late(future, abandoned=abandoned):
                        cancel_abandoned_acceptance(future, abandoned)
                    sent.add_done_callback(accepted_late)
                    try:child=self.wait(sent,3.)
                    except TimeoutError as error:
                        abandon_pending_acceptance(sent, abandoned)
                        raise PendingTaskUncertain('Task acceptance uncertain; reservation retained until Stop') from error
                    if child is None:
                        raise PendingTaskUncertain('Task acceptance unavailable; reservation retained until Stop')
                    if not child.accepted:
                        terminal_known=True
                        raise RuntimeError('Task goal rejected')
                    trace.task_goal_id.uuid = child.goal_id.uuid
                    self.capture_event(handle,'task_accepted',task_goal_uuid=bytes(child.goal_id.uuid).hex(),
                        supply_epoch=lease.supply_epoch,reservation_id=lease.reservation_id,slot_id=lease.slot_id,
                        material_type_id=lease.material_type_id,material_instance_id=lease.material_instance_id)
                    pending=child.get_result_async();cancel_sent=False;interrupted=None;cancel_deadline=None
                    self.feedback(handle,0,MissionPhase.MISSION_PHASE_PICKING,'Configured slot '+lease.slot_id,trace)
                    while not pending.done():
                        if not self.safety_allows_motion():
                            safety_preempted = True
                            interrupted = MissionExitReason.MISSION_EXIT_SAFETY_PREEMPTED
                        elif handle.is_cancel_requested:interrupted=MissionExitReason.MISSION_EXIT_CANCELED
                        elif not self.supply_is_current() or self.supply.supply_epoch != epoch:interrupted=MissionExitReason.MISSION_EXIT_SAFETY_PREEMPTED
                        if interrupted is not None and not cancel_sent:
                            name=mission_reason_name(interrupted,MissionExitReason)
                            self.log_event('warning','Configured Mission interrupted / '+name)
                            self.capture_event(handle,'mission_interrupted',exit_reason=name,
                                **self.interruption_evidence())
                            child.cancel_goal_async();cancel_sent=True;cancel_deadline=time.monotonic()+10.
                        if cancel_deadline and time.monotonic()>cancel_deadline:
                            raise PendingTaskUncertain('Task terminal uncertain after interruption; reservation retained until Stop')
                        time.sleep(.01)
                    task_result = self.wait(pending, 0.)
                    if task_result is None or task_result.status not in (4, 5, 6):
                        raise PendingTaskUncertain('Task terminal unavailable; reservation retained until Stop')
                    terminal_known = True
                    child = None
                    outcome = task_result.result
                    trace.task_status = task_result.status
                    trace.result_code = outcome.result_code
                    trace.failure_stage = outcome.failure_stage
                    trace.diagnostic_detail = outcome.diagnostic_detail
                    result.has_material_placed = True
                    result.material_placed = outcome.material_placed
                    self.capture_event(handle,'task_outcome',task_status=task_result.status,
                        task_result_code=outcome.result_code,material_placed=outcome.material_placed,
                        failure_stage=outcome.failure_stage,diagnostic_detail=outcome.diagnostic_detail,
                        supply_epoch=lease.supply_epoch,reservation_id=lease.reservation_id,slot_id=lease.slot_id,
                        material_type_id=lease.material_type_id,material_instance_id=lease.material_instance_id)
                    if outcome.result_code == TaskCode.TASK_RESULT_SAFETY_PREEMPTED or not self.safety_allows_motion():
                        safety_preempted = True
                        interrupted = MissionExitReason.MISSION_EXIT_SAFETY_PREEMPTED
                    task_succeeded = task_result.status == 4 and outcome.result_code == 0
                    if not task_succeeded:
                        detail = (f'Task failed (result_code={outcome.result_code}): '
                                  f'{outcome.failure_stage} {outcome.diagnostic_detail}')
                    if interrupted is None and task_succeeded and outcome.material_placed:
                        complete=self.command(ConfiguredSupply.Request.COMPLETE,
                            lease.supply_epoch,lease,True)
                        if not complete.success:
                            raise RuntimeError('Cell cycle retirement failed: '+complete.diagnostic_detail)
                        sequence_completed=True;lease=None
                        self.feedback(handle,0,MissionPhase.MISSION_PHASE_PLACING,'Teaching sequence ended')
                    if (outcome.material_placed and task_result.status == 6 and
                            outcome.result_code not in (TaskCode.TASK_RESULT_SUCCESS, TaskCode.TASK_RESULT_CANCELED,
                                                        TaskCode.TASK_RESULT_SAFETY_PREEMPTED)):
                        break
                    if interrupted is not None:
                        reason=interrupted;detail='Task stopped after Mission interruption';break
                    if (self.plc_admission and task_result.status == 6 and not outcome.material_placed
                            and outcome.result_code == TaskCode.TASK_RESULT_BUSY
                            and outcome.failure_stage == 'configured_supply:execution_admission'):
                        reason=MissionExitReason.MISSION_EXIT_TASK_ADMISSION_BUSY
                        detail='Task execution owner busy before BT entry';break
                    if not task_succeeded:
                        break
                    if not outcome.material_placed:raise RuntimeError('Successful Task lacks placement fact')
                else:
                    reason=MissionExitReason.MISSION_EXIT_QUANTITY_REACHED;detail='Teaching sequence completed; collision verification is independent'
            except (RuntimeError,TimeoutError) as error:
                detail=str(error)
            finally:
                if lease is not None and terminal_known:
                    try:
                        cleanup = self.command(ConfiguredSupply.Request.ABANDON,lease.supply_epoch,
                            lease,placed=result.has_material_placed and result.material_placed)
                        if not cleanup.success:
                            detail += '; reservation cleanup rejected: '+cleanup.diagnostic_detail
                    except (RuntimeError,TimeoutError) as error:detail+='; reservation cleanup failed: '+str(error)
                with self.lock:self.busy=False
            # Preserve the child outcome and placement while arbitrating the public Mission terminal.
            if safety_preempted or not self.safety_allows_motion():
                reason = MissionExitReason.MISSION_EXIT_SAFETY_PREEMPTED
                detail = 'Safety preempted Mission; ' + detail
            result.exit_reason.value=reason;result.diagnostic_detail=detail
            self.feedback(handle,0,MissionPhase.MISSION_PHASE_FINISHED,detail)
            if reason==MissionExitReason.MISSION_EXIT_CANCELED:handle.canceled()
            elif reason==MissionExitReason.MISSION_EXIT_QUANTITY_REACHED:handle.succeed()
            else:handle.abort()
            name=mission_reason_name(reason,MissionExitReason)
            self.capture_event(handle,'mission_outcome',exit_reason=name,sequence_completed=sequence_completed,diagnostic_detail=detail)
            level='info' if reason==MissionExitReason.MISSION_EXIT_QUANTITY_REACHED else 'warning'
            self.log_event(level,f'Configured Mission finished / {name}, sequence_completed={sequence_completed}: {detail}')
            return result

    rclpy.init(args=args);node=Mission();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.capture.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
