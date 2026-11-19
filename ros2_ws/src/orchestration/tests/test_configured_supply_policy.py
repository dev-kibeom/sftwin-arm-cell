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
    lease=State(supply_epoch='epoch',slot_id='slot_0',reservation_id='lease')
    state=State(supply_epoch='epoch',selected_slot_id='',reservation_id='',slot_ids=['slot_0'],occupied=[True])
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
def configured_mission(monkeypatch):
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
    lease=State(success=True,exhausted=False,supply_epoch='epoch',slot_id='slot_0',reservation_id='lease')
    def call_cell(request):
        from arm_cell_interfaces.srv import ConfiguredSupply
        requests.append(request)
        if request.command==ConfiguredSupply.Request.RESERVE:
            nodes[0].supply.selected_slot_id=lease.slot_id
            nodes[0].supply.reservation_id=lease.reservation_id
        return completed(lease)
    def send_task(goal):
        tasks.append(goal)
        result=State(status=4,result=State(result_code=0,material_placed=True,failure_stage='',diagnostic_detail=''))
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
        selected_slot_id='',reservation_id=''))
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
    assert terminal==['succeeded']
    assert [request.command for request in requests]==[ConfiguredSupply.Request.RESERVE,ConfiguredSupply.Request.COMPLETE]
    assert requests[-1].material_placed
    assert len(tasks)==1 and tasks[0].slot_id=='slot_0'
    assert feedback[-1].batch_processed_count==0
    assert not mission.busy
