import importlib.util
from pathlib import Path
from types import SimpleNamespace as State
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
