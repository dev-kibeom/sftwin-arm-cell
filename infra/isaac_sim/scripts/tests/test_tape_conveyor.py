"""Controller observations fence docking, indexing and cartridge completion.

These are state-machine tests; physical transport requires Isaac evidence.
"""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.conveyor import Conveyor


def pose(x,y=.15,z=.5):
    return NS(p=NS(x=x,y=y,z=z),r=NS(x=0.,y=math.sqrt(.5),z=0.,w=math.sqrt(.5))),NS(x=0.,y=0.,z=0.),NS(x=0.,y=0.,z=0.)


def controller():
    c=Conveyor.__new__(Conveyor)
    c.settings=dict(max_cartridges=8,movement_timeout_s=90.,speed_m_s=.04,acceleration_m_s2=.06,
        position_tolerance_m=.002,settle_s=.25,outfeed_speed_m_s=.06,outfeed_pitch_m=.18,inlet_front_x_m=1.12)
    c.layout=dict(supply_center_m=[.53,.15,.5],cartridge=dict(slot_count=2,slot_spacing_m=.03),place=dict(first_center_xy_m=[.3,-.25]))
    c.state='idle';c.queued=c.accepted=c.finished=0
    c.elapsed=c.settle=c._speed=0.;c._command=c._target=.53
    c.ordinal=0;c.active_roll_paths=['r0','r1'];c.carrier_path='carrier'
    c.drive=NS(GetTargetPositionAttr=lambda:NS(Set=lambda value:None))
    c._surface=lambda **kw:None
    c.awakened=[];c._wake_roll=c.awakened.append
    c.awakened=[];c._wake_roll=c.awakened.append
    c.observations=dict(carrier=pose(.53),r0=pose(.53),r1=pose(.56))
    c._measure=lambda path,frame='infeed':c.observations.get(path)
    return c


def test_docking_uses_drive_feedback_without_reading_material_ground_truth():
    c=controller();c.state='infeed'
    def measured(path, frame='infeed'):
        assert path == 'carrier', 'material truth entered conveyor control'
        return pose(.53)
    c._measure=measured
    c.on_physics_step(.3)
    assert c.dock_ready


def test_outfeed_uses_belt_travel_time_without_measuring_released_material():
    c=controller();c.state='docked'
    c._measure=lambda *args: (_ for _ in ()).throw(AssertionError('material truth queried'))
    c.release_to_outfeed('r0')
    c.on_physics_step(.1)
    assert c.state=='outfeed'
    c.on_physics_step(c.settings['outfeed_pitch_m']/c.settings['outfeed_speed_m_s'])
    assert c.state=='indexing' and c.ordinal==1
    assert c._target==pytest.approx(.5)


def test_timeout_and_busy_requests_do_not_report_completion():
    c=controller()
    assert not c.enqueue_cartridges(True)['accepted']
    assert c.enqueue_cartridges(2)['accepted']
    assert not c.enqueue_cartridges(1)['accepted']
    c.state='infeed';c.on_physics_step(91.)
    assert c.state=='failed' and c.finished==0
    assert not c.enqueue_cartridges(1)['accepted']


def test_missing_simulated_payload_does_not_block_timed_outfeed():
    c=controller();c.state='docked'
    def unavailable(path): raise RuntimeError('simulated payload unavailable')
    c._wake_roll=unavailable
    c.release_to_outfeed('missing-roll')
    assert c.state=='outfeed'


def test_belt_commands_stop_both_directions_on_idle_and_timeout():
    c=controller()
    commands=[]
    c.belts=NS(set_speeds=lambda **speeds:commands.append(speeds))
    c._surface=Conveyor._surface.__get__(c)
    c._surface(infeed=-.04, outfeed=.06)
    assert commands[-1]==dict(infeed=-.04, outfeed=.06)
    c.on_physics_step(.1)
    assert commands[-1]==dict(infeed=0., outfeed=0.)
    c.state='infeed'
    c.on_physics_step(91.)
    assert c.state=='failed'
    assert commands[-1]==dict(infeed=0., outfeed=0.)
    c.close()
    assert commands[-1]==dict(infeed=0., outfeed=0.)
