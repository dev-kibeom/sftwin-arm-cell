#!/usr/bin/env python3
"""Configured-source Mission lifecycle. Vision acquisition is not part of this work source."""
import threading
import time


def safety_permits(state, received_at, now):
    return bool(state is not None and now-received_at <= .5 and state.valid and
                state.required_inputs_fresh and state.motion_envelope_valid and
                state.motion_capability.value == 2 and state.max_velocity_scale > 0 and state.max_acceleration_scale > 0)


def supply_permits(state, received_at, now, supply_id, material_id):
    return bool(state is not None and now-received_at <= .5 and state.valid and
                state.supply_id == supply_id and state.material_id == material_id and state.supply_epoch and
                len(state.slot_ids) == len(state.occupied))


def lease_matches(state, lease):
    return bool(state is not None and state.supply_epoch == lease.supply_epoch and
                state.selected_slot_id == lease.slot_id and state.reservation_id == lease.reservation_id and
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
    from rclpy.executors import MultiThreadedExecutor
    from arm_cell_interfaces.action import ExecuteCycle, ExecuteConfiguredGrasp
    from arm_cell_interfaces.srv import ConfiguredSupply
    from arm_cell_interfaces.msg import ConfiguredSupplyState, SafetyState, MissionExitReason, MissionPhase

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
            self.server=None
            self.startup_timer=self.create_timer(.1,self.advertise_when_ready)

        def advertise_when_ready(self):
            if self.server is None and self.task.server_is_ready() and self.cell.service_is_ready():
                self.server=ActionServer(self,ExecuteCycle,'/orchestration/execute_cycle',execute_callback=self.execute,
                    goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
                self.startup_timer.cancel()

        def on_safety(self,state):self.safety=state;self.safety_at=time.monotonic()
        def on_supply(self,state):self.supply=state;self.supply_at=time.monotonic()

        def ready(self):
            now=time.monotonic()
            return (safety_permits(self.safety,self.safety_at,now) and
                supply_permits(self.supply,self.supply_at,now,self.supply_id,self.target))

        def accept(self,goal):
            with self.lock:
                if (self.busy or not self.ready() or self.supply.reservation_id or goal.target_id != self.target or
                        not 0 < goal.quantity or any(goal.delivery_id.uuid) or not self.task.server_is_ready() or not self.cell.service_is_ready()):
                    return GoalResponse.REJECT
                self.busy=True
            return GoalResponse.ACCEPT

        @staticmethod
        def wait(future,timeout):
            deadline=time.monotonic()+timeout
            while not future.done():
                if time.monotonic()>=deadline:raise TimeoutError('ROS operation result deadline exceeded')
                time.sleep(.01)
            return future.result()

        def command(self,command,epoch='',slot='',reservation='',placed=False):
            request=ConfiguredSupply.Request();request.command=command;request.supply_id=self.supply_id
            request.supply_epoch=epoch;request.slot_id=slot;request.reservation_id=reservation;request.material_placed=placed
            return self.wait(self.cell.call_async(request),3.5)

        def feedback(self,handle,count,phase,detail):
            message=ExecuteCycle.Feedback();message.phase.value=phase
            message.delivery_id=handle.request.delivery_id;message.quantity=handle.request.quantity
            message.batch_processed_count=count;message.diagnostic_detail=detail
            handle.publish_feedback(message)

        def execute(self,handle):
            result=ExecuteCycle.Result();count=0;lease=None;child=None;terminal_known=True
            reason=MissionExitReason.MISSION_EXIT_MOTION_ERROR;detail='configured task did not complete'
            try:
                epoch=self.supply.supply_epoch
                while count < handle.request.quantity:
                    if handle.is_cancel_requested:
                        reason=MissionExitReason.MISSION_EXIT_CANCELED;detail='Canceled before slot reservation';break
                    if not self.ready() or self.supply.supply_epoch != epoch:
                        reason=MissionExitReason.MISSION_EXIT_SAFETY_PREEMPTED;detail='Safety or configured supply became unavailable';break
                    lease=self.command(ConfiguredSupply.Request.RESERVE,epoch)
                    if lease.supply_epoch != epoch:
                        raise RuntimeError('Cell session changed during reservation')
                    if lease.exhausted:
                        reason=MissionExitReason.MISSION_EXIT_QUANTITY_SHORTFALL;detail='Configured supply exhausted before requested quantity';lease=None;break
                    if not lease.success:
                        raise RuntimeError('Cell reservation rejected: '+lease.diagnostic_detail)
                    deadline=time.monotonic()+1.
                    while not (self.ready() and lease_matches(self.supply,lease)):
                        if time.monotonic()>deadline or self.supply.supply_epoch != epoch or handle.is_cancel_requested:
                            raise RuntimeError('Cell reservation state not confirmed')
                        time.sleep(.01)
                    goal=ExecuteConfiguredGrasp.Goal();goal.supply_id=self.supply_id;goal.slot_id=lease.slot_id
                    goal.supply_epoch=lease.supply_epoch;goal.reservation_id=lease.reservation_id
                    abandoned=threading.Event()
                    sent=self.task.send_goal_async(goal)
                    def accepted_late(future, abandoned=abandoned):
                        cancel_abandoned_acceptance(future, abandoned)
                    sent.add_done_callback(accepted_late)
                    try:child=self.wait(sent,3.)
                    except TimeoutError as error:
                        terminal_known=False
                        abandon_pending_acceptance(sent, abandoned)
                        raise PendingTaskUncertain('Task acceptance uncertain; reservation retained until Stop') from error
                    if child is None or not child.accepted:raise RuntimeError('Task goal rejected')
                    terminal_known=False
                    pending=child.get_result_async();cancel_sent=False;interrupted=None;cancel_deadline=None
                    self.feedback(handle,count,MissionPhase.MISSION_PHASE_PICKING,'Configured slot '+lease.slot_id)
                    while not pending.done():
                        if handle.is_cancel_requested:interrupted=MissionExitReason.MISSION_EXIT_CANCELED
                        elif not self.ready() or self.supply.supply_epoch != epoch:interrupted=MissionExitReason.MISSION_EXIT_SAFETY_PREEMPTED
                        if interrupted is not None and not cancel_sent:
                            now=time.monotonic()
                            self.get_logger().warning(
                                f'Configured Mission interrupted: reason={interrupted} '
                                f'safety_age_s={now-self.safety_at:.3f} '
                                f'supply_age_s={now-self.supply_at:.3f} '
                                f'safety={self.safety} supply={self.supply}')
                            child.cancel_goal_async();cancel_sent=True;cancel_deadline=time.monotonic()+10.
                        if cancel_deadline and time.monotonic()>cancel_deadline:
                            raise PendingTaskUncertain('Task terminal uncertain after interruption; reservation retained until Stop')
                        time.sleep(.01)
                    task_result=pending.result();terminal_known=True;child=None
                    outcome=task_result.result
                    if outcome.material_placed:
                        deadline=time.monotonic()+2.
                        while True:
                            complete=self.command(ConfiguredSupply.Request.COMPLETE,lease.supply_epoch,lease.slot_id,lease.reservation_id,True)
                            if complete.success:count+=1;lease=None;break
                            if time.monotonic()>deadline:raise RuntimeError('Cell receiving confirmation failed: '+complete.diagnostic_detail)
                            time.sleep(.05)
                        self.feedback(handle,count,MissionPhase.MISSION_PHASE_PLACING,'Physically received and released')
                    if interrupted is not None:
                        reason=interrupted;detail='Task stopped after Mission interruption';break
                    if task_result.status != 4 or outcome.result_code != 0:
                        detail='Task failed: '+outcome.failure_stage+' '+outcome.diagnostic_detail;break
                    if not outcome.material_placed:raise RuntimeError('Successful Task lacks placement fact')
                else:
                    reason=MissionExitReason.MISSION_EXIT_QUANTITY_REACHED;detail='Requested configured supply quantity completed'
            except (RuntimeError,TimeoutError) as error:
                detail=str(error)
            finally:
                if lease is not None and terminal_known:
                    try:self.command(ConfiguredSupply.Request.ABANDON,lease.supply_epoch,lease.slot_id,lease.reservation_id)
                    except (RuntimeError,TimeoutError) as error:detail+='; reservation cleanup failed: '+str(error)
                with self.lock:self.busy=False
            result.exit_reason.value=reason;result.diagnostic_detail=detail
            self.feedback(handle,count,MissionPhase.MISSION_PHASE_FINISHED,detail)
            if reason==MissionExitReason.MISSION_EXIT_CANCELED:handle.canceled()
            elif reason==MissionExitReason.MISSION_EXIT_QUANTITY_REACHED:handle.succeed()
            else:handle.abort()
            self.get_logger().info(f'Configured Mission result={reason}, processed={count}: {detail}')
            return result

    rclpy.init(args=args);node=Mission();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    finally:executor.shutdown();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
