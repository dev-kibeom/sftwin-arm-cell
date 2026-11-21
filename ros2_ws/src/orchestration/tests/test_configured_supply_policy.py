import importlib.util
from pathlib import Path
from types import SimpleNamespace as State
import pytest
spec=importlib.util.spec_from_file_location('configured_mission',Path(__file__).resolve().parents[1]/'scripts/configured_supply_mission.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_missing_stale_or_restricted_safety_never_permits_configured_work():
    safe=State(valid=True,required_inputs_fresh=True,motion_envelope_valid=True,
               motion_capability=State(value=2),max_velocity_scale=.5,max_acceleration_scale=.5)
    assert module.safety_permits(safe,10.,10.1)
    assert not module.safety_permits(None,10.,10.1)
    assert not module.safety_permits(safe,10.,11.)
    safe.motion_capability.value=1
    assert not module.safety_permits(safe,10.,10.1)
    safe.motion_capability.value=2;safe.required_inputs_fresh=False
    assert not module.safety_permits(safe,10.,10.1)


def test_configured_inventory_requires_matching_source_and_fresh_valid_epoch():
    state=State(valid=True,supply_id='cartridge',material_id='TapeRoll17',supply_epoch='new',slot_ids=['slot_0'],occupied=[True])
    assert module.supply_permits(state,10.,10.1,'cartridge','TapeRoll17')
    assert not module.supply_permits(state,10.,11.,'cartridge','TapeRoll17')
    assert not module.supply_permits(state,10.,10.1,'other','TapeRoll17')
    state.supply_epoch=''
    assert not module.supply_permits(state,10.,10.1,'cartridge','TapeRoll17')


def test_acceptance_before_or_after_timeout_is_canceled():
    from concurrent.futures import Future
    from threading import Event
    for accepted_before_timeout in (True, False):
        cancellations=[]
        goal=State(accepted=True,cancel_goal_async=lambda:cancellations.append(True))
        future=Future();abandoned=Event()
        future.add_done_callback(lambda f:module.cancel_abandoned_acceptance(f,abandoned))
        if accepted_before_timeout:future.set_result(goal)
        module.abandon_pending_acceptance(future,abandoned)
        if not accepted_before_timeout:future.set_result(goal)
        assert cancellations


def test_task_handoff_waits_for_observed_occupied_lease():
    lease=State(supply_epoch='epoch',slot_id='slot_0',reservation_id='lease',material_type_id='A',material_instance_id='1')
    state=State(supply_epoch='epoch',selected_slot_id='',reservation_id='',material_type_id='A',material_instance_id='1',slot_ids=['slot_0'],occupied=[True])
    assert not module.lease_matches(state,lease)
    state.selected_slot_id='slot_0';state.reservation_id='lease'
    assert module.lease_matches(state,lease)
    state.occupied=[False]
    assert not module.lease_matches(state,lease)


def test_selected_diagnostics_preserves_mission_causality_and_survives_bad_storage(tmp_path):
    import json
    capture=module.ConfiguredMissionDiagnostics(tmp_path,enabled=True)
    capture.emit('reservation_confirmed',execute_cycle_id='mission',task_goal_uuid='task',
                 reservation_id='lease',supply_epoch='epoch',slot_id='slot_0')
    assert capture.close()
    records=[json.loads(line) for path in tmp_path.glob('*.jsonl') for line in path.read_text().splitlines()]
    assert records[0]['execute_cycle_id']=='mission'
    assert records[0]['task_goal_uuid']=='task'
    assert records[0]['reservation_id']=='lease'
    disabled=module.ConfiguredMissionDiagnostics(tmp_path/'disabled',enabled=False)
    disabled.emit('unused');assert disabled.close()
    assert not (tmp_path/'disabled').exists()
    bad=tmp_path/'file';bad.write_text('not a directory')
    capture=module.ConfiguredMissionDiagnostics(bad,enabled=True)
    capture.emit('mission_outcome',exit_reason='MISSION_EXIT_MOTION_ERROR')
    assert capture.close()
    assert capture.failed_count>0


@pytest.fixture
def configured_mission(monkeypatch, request):
    """Run Mission policy with in-process ROS transport substitutes; no DDS or motion."""
    import rclpy
    import rclpy.node
    import rclpy.action
    import rclpy.executors
    nodes=[];requests=[];tasks=[]
    def completed(value):
        from concurrent.futures import Future
        future=Future();future.set_result(value);return future
    class Node:
        def __init__(self,*args):nodes.append(self)
        def declare_parameter(self,name,default):return State(value=default)
        def create_subscription(self,*args,**kwargs):pass
        def create_timer(self,*args):return State(cancel=lambda:None)
        def create_client(self,*args,**kwargs):return State(service_is_ready=lambda:True,call_async=call_cell)
        def get_logger(self):return State(info=lambda message:None,warning=lambda message:None,debug=lambda message:None)
        def destroy_node(self):pass
    class Executor:
        def __init__(self,**kwargs):pass
        def add_node(self,node):pass
        def spin(self):pass
        def shutdown(self):pass
    lease=State(success=True,exhausted=False,supply_epoch='epoch',slot_id='slot_0',reservation_id='lease',
                material_type_id='A',material_instance_id='42')
    def call_cell(request):
        from arm_cell_interfaces.srv import ConfiguredSupply
        requests.append(request)
        if request.command==ConfiguredSupply.Request.RESERVE:
            nodes[0].supply.selected_slot_id=lease.slot_id
            nodes[0].supply.reservation_id=lease.reservation_id
        return completed(lease)
    def send_task(goal):
        tasks.append(goal)
        result=State(status=getattr(request, 'param', 4),result=State(result_code=0,material_placed=True,failure_stage='',diagnostic_detail=''))
        return completed(State(accepted=True,goal_id=State(uuid=[1]*16),get_result_async=lambda:completed(result)))
    monkeypatch.setattr(rclpy,'init',lambda **kwargs:None)
    monkeypatch.setattr(rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(rclpy.node,'Node',Node)
    monkeypatch.setattr(rclpy.executors,'MultiThreadedExecutor',Executor)
    monkeypatch.setattr(rclpy.action,'ActionClient',lambda *args,**kwargs:
        State(server_is_ready=lambda:True,send_goal_async=send_task))
    module.main()
    mission=nodes[0]
    mission.on_safety(State(valid=True,required_inputs_fresh=True,motion_envelope_valid=True,
        motion_capability=State(value=2),max_velocity_scale=.5,max_acceleration_scale=.5))
    mission.on_supply(State(valid=True,supply_id=mission.supply_id,material_id=mission.target,
        supply_epoch='epoch',slot_ids=['slot_0','slot_1'],occupied=[True,True],
        selected_slot_id='',reservation_id='',material_type_id='A',material_instance_id='42'))
    return mission,requests,tasks


@pytest.mark.parametrize('quantity',[0,2,3,15,100])
def test_configured_mission_rejects_non_unit_quantity_before_any_work(configured_mission,quantity):
    from arm_cell_interfaces.action import ExecuteCycle
    from rclpy.action import GoalResponse
    mission,requests,tasks=configured_mission
    goal=ExecuteCycle.Goal();goal.target_id=mission.target;goal.quantity=quantity
    assert mission.accept(goal)==GoalResponse.REJECT
    assert not mission.busy
    assert not requests and not tasks


def test_unit_request_runs_one_sequence_without_material_count(configured_mission):
    from arm_cell_interfaces.action import ExecuteCycle
    from arm_cell_interfaces.msg import MissionExitReason
    from arm_cell_interfaces.srv import ConfiguredSupply
    from rclpy.action import GoalResponse
    mission,requests,tasks=configured_mission
    goal=ExecuteCycle.Goal();goal.target_id=mission.target;goal.quantity=1
    assert mission.accept(goal)==GoalResponse.ACCEPT
    assert not requests and not tasks
    feedback=[];terminal=[]
    handle=State(request=goal,goal_id=State(uuid=[2]*16),is_cancel_requested=False,
        publish_feedback=feedback.append,succeed=lambda:terminal.append('succeeded'),
        abort=lambda:terminal.append('aborted'),canceled=lambda:terminal.append('canceled'))
    result=mission.execute(handle)
    assert result.exit_reason.value==MissionExitReason.MISSION_EXIT_QUANTITY_REACHED
    assert result.has_material_placed and result.material_placed
    trace = result.configured_task
    assert trace.material_instance_id == '42' and trace.reservation_id == 'lease'
    assert bytes(trace.task_goal_id.uuid) == bytes([1]*16)
    assert (trace.task_status, trace.result_code) == (4, 0)
    assert any(f.configured_task.reservation_id == 'lease' for f in feedback)
    assert terminal==['succeeded']
    assert [request.command for request in requests]==[ConfiguredSupply.Request.RESERVE,ConfiguredSupply.Request.COMPLETE]
    assert requests[-1].material_placed
    assert len(tasks)==1 and tasks[0].slot_id=='slot_0'
    for message in (tasks[0], requests[-1]):
        assert (message.material_type_id, message.material_instance_id) == ('A', '42')
    assert feedback[-1].batch_processed_count==0
    assert not mission.busy


@pytest.mark.parametrize('configured_mission', [0], indirect=True)
def test_unknown_task_terminal_keeps_reservation_and_has_no_release_fact(configured_mission):
    from arm_cell_interfaces.action import ExecuteCycle
    from arm_cell_interfaces.srv import ConfiguredSupply
    mission, requests, tasks = configured_mission
    goal = ExecuteCycle.Goal(target_id=mission.target, quantity=1)
    mission.accept(goal)
    terminal = []
    handle = State(request=goal, goal_id=State(uuid=[2]*16), is_cancel_requested=False,
                   publish_feedback=lambda _: None, abort=lambda: terminal.append('aborted'))
    result = mission.execute(handle)
    assert terminal == ['aborted'] and not result.has_material_placed
    assert [req.command for req in requests] == [ConfiguredSupply.Request.RESERVE]
    assert len(tasks) == 1


def test_motion_admission_uses_owner_state_without_requiring_stop_evidence_for_idle():
    from arm_cell_interfaces.msg import MotionStatus
    status = MotionStatus(execution_state=MotionStatus.MOTION_STATE_IDLE)
    assert module.motion_allows_admission(status, 5., 5.1)
    assert not module.motion_allows_admission(status, 5., 6.1)
    for name in ('EXECUTING', 'STOPPING', 'EMERGENCY_STOPPED', 'FAULTED', 'UNKNOWN'):
        status.execution_state = getattr(MotionStatus, 'MOTION_STATE_' + name)
        assert not module.motion_allows_admission(status, 5., 5.1)
    status.execution_state = MotionStatus.MOTION_STATE_STOPPED
    assert not module.motion_allows_admission(status, 5., 5.1)
    status.backend_inactivity_confirmed = True
    assert module.motion_allows_admission(status, 5., 5.1)
    status.has_active_execution = True
    assert not module.motion_allows_admission(status, 5., 5.1)


def test_reservation_rejection_never_releases_another_lease(configured_mission):
    from arm_cell_interfaces.action import ExecuteCycle
    from arm_cell_interfaces.srv import ConfiguredSupply
    mission, _, tasks = configured_mission
    commands = []

    def reject(command, *args, **kwargs):
        commands.append(command)
        return State(success=False, supply_epoch='epoch', exhausted=False,
                     diagnostic_detail='already reserved', reservation_id='other', slot_id='slot_0')

    mission.command = reject
    goal = ExecuteCycle.Goal(target_id=mission.target, quantity=1)
    mission.accept(goal)
    terminal = []
    handle = State(request=goal, goal_id=State(uuid=[2]*16), is_cancel_requested=False,
                   publish_feedback=lambda _: None, abort=lambda: terminal.append('aborted'))
    result = mission.execute(handle)
    assert terminal == ['aborted'] and not result.has_material_placed
    assert commands == [ConfiguredSupply.Request.RESERVE] and not tasks


@pytest.mark.parametrize('placed,expected_terminal', [(True, 'aborted'), (False, 'canceled')])
def test_interruption_preserves_observed_task_outcome(configured_mission, placed, expected_terminal):
    from concurrent.futures import Future
    from arm_cell_interfaces.action import ExecuteCycle, ExecuteConfiguredGrasp
    from arm_cell_interfaces.msg import MissionExitReason, MotionTaskResultCode
    from arm_cell_interfaces.srv import ConfiguredSupply
    mission, requests, _ = configured_mission
    goal = ExecuteCycle.Goal(target_id=mission.target, quantity=1)
    mission.accept(goal)
    terminal = []
    handle = State(request=goal, goal_id=State(uuid=[2]*16), is_cancel_requested=False,
                   publish_feedback=lambda _: None, abort=lambda: terminal.append('aborted'),
                   canceled=lambda: terminal.append('canceled'))
    result_future = Future()
    outcome = ExecuteConfiguredGrasp.Result(
        result_code=(MotionTaskResultCode.TASK_RESULT_PATH_OBSTRUCTED if placed
                     else MotionTaskResultCode.TASK_RESULT_CANCELED),
        material_placed=placed, failure_stage='RETREAT' if placed else 'TRANSFER')

    def send_task(_):
        handle.is_cancel_requested = True
        sent = Future()
        sent.set_result(State(accepted=True, goal_id=State(uuid=[1]*16),
                              get_result_async=lambda: result_future,
                              cancel_goal_async=lambda: result_future.set_result(
                                  State(status=6 if placed else 5, result=outcome))))
        return sent

    mission.task.send_goal_async = send_task
    result = mission.execute(handle)
    assert terminal == [expected_terminal]
    assert result.has_material_placed and result.material_placed == placed
    assert result.exit_reason.value == (MissionExitReason.MISSION_EXIT_MOTION_ERROR if placed
                                        else MissionExitReason.MISSION_EXIT_CANCELED)
    assert [req.command for req in requests] == [ConfiguredSupply.Request.RESERVE, ConfiguredSupply.Request.ABANDON]
    assert requests[-1].material_placed == placed
    if placed:
        assert 'RETREAT' in result.diagnostic_detail


@pytest.mark.parametrize('preemption', ['while_running', 'with_terminal', 'during_cleanup'])
def test_mission_safety_precedence_preserves_placed_task_failure(configured_mission, preemption):
    from concurrent.futures import Future
    from arm_cell_interfaces.action import ExecuteCycle, ExecuteConfiguredGrasp
    from arm_cell_interfaces.msg import MissionExitReason, MotionTaskResultCode
    from arm_cell_interfaces.srv import ConfiguredSupply
    mission, requests, _ = configured_mission
    goal = ExecuteCycle.Goal(target_id=mission.target, quantity=1)
    mission.accept(goal)
    terminals = []
    handle = State(request=goal, goal_id=State(uuid=[2]*16), is_cancel_requested=False,
                   publish_feedback=lambda _: None, abort=lambda: terminals.append('aborted'))
    failure = ExecuteConfiguredGrasp.Result(result_code=MotionTaskResultCode.TASK_RESULT_PATH_OBSTRUCTED,
                                            failure_stage='RETREAT', diagnostic_detail='blocked', material_placed=True)
    terminal = State(status=6, result=failure)
    pending = Future()
    events = []
    mission.capture_event = lambda _, event, **facts: events.append((event, facts))

    def cancel():
        mission.safety.motion_capability.value = 2  # Safety had preempted before the Task terminal arrived.
        pending.set_result(terminal)

    def send_task(_):
        if preemption != 'during_cleanup':
            mission.safety.motion_capability.value = 1
        if preemption == 'while_running':
            handle.is_cancel_requested = True
        else:
            pending.set_result(terminal)
        sent = Future()
        sent.set_result(State(accepted=True, goal_id=State(uuid=[1]*16), get_result_async=lambda: pending,
                              cancel_goal_async=cancel))
        return sent

    command = mission.command

    def cell_command(cmd, *args, **kwargs):
        if cmd == ConfiguredSupply.Request.ABANDON:
            if preemption == 'during_cleanup':
                mission.safety.motion_capability.value = 1
            elif preemption == 'with_terminal':
                mission.safety.motion_capability.value = 2
        return command(cmd, *args, **kwargs)

    mission.task.send_goal_async = send_task
    mission.command = cell_command
    result = mission.execute(handle)
    assert result.exit_reason.value == MissionExitReason.MISSION_EXIT_SAFETY_PREEMPTED
    assert terminals == ['aborted'] and result.has_material_placed and result.material_placed
    assert requests[-1].command == ConfiguredSupply.Request.ABANDON and requests[-1].material_placed
    assert f'result_code={failure.result_code}' in result.diagnostic_detail
    assert 'RETREAT' in result.diagnostic_detail and 'blocked' in result.diagnostic_detail
    task_outcome, = [facts for event, facts in events if event == 'task_outcome']
    assert task_outcome['task_result_code'] == failure.result_code and task_outcome['material_placed']


def test_reconnected_supply_receives_only_the_original_terminal_cleanup(configured_mission):
    from arm_cell_interfaces.srv import ConfiguredSupply
    mission, requests, tasks = configured_mission
    mission.plc_admission = True
    lease = State(slot_id='slot', reservation_id='lease', material_type_id='A', material_instance_id='stream/1')
    wait = mission.wait
    def lost_first_response(future, timeout):
        if len(requests) == 1:
            raise TimeoutError('Disconnected during terminal cleanup')
        return wait(future, timeout)
    mission.wait = lost_first_response
    mission.cell.wait_for_service = lambda timeout_sec: True
    response = mission.command(ConfiguredSupply.Request.ABANDON, 'epoch', lease, placed=False)
    assert response.success and not tasks
    assert len(requests) == 2 and requests[0] == requests[1]
    assert requests[-1].reservation_id == 'lease' and not requests[-1].material_placed


def test_supply_delay_reports_transitions_and_counts_polls():
    delay = module.SupplyDelay()
    events = []
    for age in (.49, .5, .8, 5., 6., 0., .1):
        delay.report(age, lambda level, message: events.append((level, message)))
    assert [level for level, _ in events] == ['warning', 'error', 'info']
    assert '4' in events[-1][1]


@pytest.mark.parametrize('field,value', [
    ('reservation_id', ''), ('supply_epoch', 'replacement'),
    ('material_instance_id', 'stream/other'), ('valid', False)])
def test_confirmed_conveyor_revocation_is_latched(configured_mission, field, value):
    from copy import copy
    mission, _, _ = configured_mission
    state = copy(mission.supply)
    state.reservation_id, state.selected_slot_id = 'lease', 'slot_0'
    mission.active_lease = State(
        supply_epoch=state.supply_epoch, slot_id='slot_0', reservation_id='lease',
        material_type_id=state.material_type_id, material_instance_id=state.material_instance_id)
    pending = copy(state)
    pending.valid, pending.observation_pending = False, True
    mission.on_supply(pending)
    assert not mission.revoked and not mission.supply_is_current()
    mission.on_supply(state)
    assert not mission.revoked and mission.supply_is_current()
    changed = copy(state)
    setattr(changed, field, value)
    mission.on_supply(changed)
    mission.on_supply(state)
    assert mission.revoked


def test_supply_delay_emits_all_severities_to_ros_logger(configured_mission, capfd):
    from rclpy.logging import get_logger
    mission, _, _ = configured_mission
    logger = get_logger('supply_delay_regression')
    mission.get_logger = lambda: logger
    for age in (0.5, 5.0, 0.):
        mission.supply_delay.report(age, mission.log_event)
    log = capfd.readouterr().err
    assert log.count('observation delayed') == 2
    assert '[WARN]' in log and '[ERROR]' in log and '[INFO]' in log
