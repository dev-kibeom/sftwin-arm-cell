"""Collision verification observes a cycle without feeding back into control."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.cycle_evidence import CycleEvidence


def test_complete_collision_free_cycle_passes_and_contact_fails():
    records=[]
    e=CycleEvidence(records.append, session_id='session')
    e.start('first', supply_epoch='epoch', slot_id='slot_0')
    e.physics_step(1)
    e.observe([], step_id=1)
    e.finish(True)
    assert records[-1]['verification']=='PASS'
    assert records[-1]['session_id']=='session'
    e.start('second', supply_epoch='epoch', slot_id='slot_1')
    e.physics_step(2)
    e.observe([{'actor0':'robot','actor1':'fixture'}], step_id=2)
    e.finish(True)
    assert records[-1]['verification']=='FAIL'
    assert records[-1]['sequence_completed'] is True


def test_missing_observations_and_interrupted_cycles_never_pass():
    records=[];e=CycleEvidence(records.append)
    e.start('missing');e.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'
    e.start('gap');e.physics_step(1);e.observe(None, step_id=1);e.physics_step(2);e.observe([], step_id=2);e.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'
    e.start('interrupted');e.physics_step(1);e.observe([], step_id=1);e.finish(False)
    assert records[-1]['verification']=='FAIL'


def test_broken_diagnostic_sink_does_not_raise_into_runtime():
    def broken(record): raise OSError('disk unavailable')
    e=CycleEvidence(broken);e.start('cycle');e.physics_step(1);e.observe([], step_id=1);e.finish(True)
    assert e.active is None


def test_stopped_observation_callback_cannot_pass_after_one_clean_step():
    records=[];e=CycleEvidence(records.append)
    e.start('tail-gap')
    e.physics_step(10);e.observe([], step_id=10)
    e.physics_step(11)
    e.physics_step(12)
    # An empty terminal drain is not a Physics Step observation.
    e.observe([], physics_step=False)
    e.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'
    assert records[-1]['executed_steps']==3
    assert records[-1]['observed_steps']==1


def test_observation_resuming_after_a_missing_middle_step_does_not_restore_coverage():
    records=[];e=CycleEvidence(records.append)
    e.start('middle-gap')
    e.physics_step(20);e.observe([], step_id=20)
    e.physics_step(21)
    e.physics_step(22);e.observe([], step_id=22)
    e.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'
    assert not records[-1]['coverage']['contact_observation_complete']


def test_duplicate_observations_cannot_replace_a_missing_step():
    records=[];e=CycleEvidence(records.append)
    e.start('duplicate')
    e.physics_step(30);e.observe([], step_id=30)
    e.observe([], step_id=30)
    e.physics_step(31)
    e.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'
    assert records[-1]['observed_steps']==1


def test_observation_for_an_older_step_cannot_cover_current_step():
    records=[];e=CycleEvidence(records.append)
    e.start('late')
    e.physics_step(40);e.observe([], step_id=40)
    e.physics_step(41);e.observe([], step_id=40)
    e.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'


def test_all_executed_steps_observed_once_can_pass_after_a_prior_incomplete_cycle():
    records=[];e=CycleEvidence(records.append)
    e.start('incomplete');e.physics_step(50);e.finish(True)
    e.start('complete')
    for step_id in range(51,54):
        e.physics_step(step_id);e.observe([], step_id=step_id)
    e.finish(True)
    record=records[-1]
    assert record['verification']=='PASS'
    assert record['executed_steps']==record['observed_steps']==3
    assert record['first_step_id']==51 and record['last_executed_step_id']==53


def test_missing_final_observation_is_reported_as_incomplete_coverage():
    records=[];e=CycleEvidence(records.append)
    e.start('last-step')
    e.physics_step(60);e.observe([], step_id=60)
    e.physics_step(61)
    e.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'
    assert not records[-1]['coverage_complete']
