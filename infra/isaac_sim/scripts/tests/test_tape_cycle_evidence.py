"""Collision verification observes a cycle without feeding back into control."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.cycle_evidence import CycleEvidence


def test_complete_collision_free_cycle_passes_and_contact_fails():
    records=[]
    e=CycleEvidence(records.append, session_id='session')
    e.start('first', supply_epoch='epoch', slot_id='slot_0')
    e.observe([])
    e.finish(True)
    assert records[-1]['verification']=='PASS'
    assert records[-1]['session_id']=='session'
    e.start('second', supply_epoch='epoch', slot_id='slot_1')
    e.observe([{'actor0':'robot','actor1':'fixture'}])
    e.finish(True)
    assert records[-1]['verification']=='FAIL'
    assert records[-1]['sequence_completed'] is True


def test_missing_observations_and_interrupted_cycles_never_pass():
    records=[];e=CycleEvidence(records.append)
    e.start('missing');e.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'
    e.start('gap');e.observe(None);e.observe([]);e.finish(True)
    assert records[-1]['verification']=='UNAVAILABLE'
    e.start('interrupted');e.observe([]);e.finish(False)
    assert records[-1]['verification']=='FAIL'


def test_broken_diagnostic_sink_does_not_raise_into_runtime():
    def broken(record): raise OSError('disk unavailable')
    e=CycleEvidence(broken);e.start('cycle');e.observe([]);e.finish(True)
    assert e.active is None
