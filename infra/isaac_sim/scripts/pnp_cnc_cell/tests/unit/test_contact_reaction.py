import pytest
from gripper_runtime.contact_reaction import finger_reactions


def contact(side, impulse, target='/material'):
    return dict(actor0='/robot/gripper_robotiq_85_' + side + '_finger_tip_link',
                actor1=target, lost=False,
                points=[dict(impulse=impulse, normal=(1., 0., 0.))])


def test_opposed_finger_forces_do_not_cancel():
    assert finger_reactions([contact('left', (0.1, 0., 0.)),
                             contact('right', (-0.2, 0., 0.))], 0.1) == pytest.approx((1., 2.))


def test_different_objects_or_missing_step_cannot_certify_grasp():
    pairs = [contact('left', (0.1, 0., 0.)), contact('right', (-0.1, 0., 0.), '/tray')]
    assert finger_reactions(pairs, 0.1) == (0., 0.)
    assert finger_reactions(pairs, 0.) is None
    assert finger_reactions(None, 0.1) is None


def test_tangential_impulse_and_lost_contacts_do_not_supply_reaction():
    left = contact('left', (0., 1., 0.))
    right = contact('right', (1., 0., 0.))
    right['lost'] = True
    assert finger_reactions([left, right], 0.1) == (0., 0.)


def test_selected_robot_body_and_reference_are_required():
    pairs=[contact('left',(0.1,0.,0.)),contact('right',(-0.1,0.,0.))]
    fingers=tuple('/robot/gripper_robotiq_85_'+side+'_finger_tip_link' for side in ('left','right'))
    assert finger_reactions(pairs,0.1,target_path='/other',fingertip_paths=fingers)==(0.,0.)
    for pair in pairs:pair['actor0']=pair['actor0'].replace('/robot/','/other_robot/')
    assert finger_reactions(pairs,0.1,target_path='/material',fingertip_paths=fingers)==(0.,0.)


def test_reaction_uses_current_physics_step_dt():
    pairs=[contact('left',(0.01,0.,0.)),contact('right',(-0.01,0.,0.))]
    assert finger_reactions(pairs,0.1)==pytest.approx((0.1,0.1))
    assert finger_reactions(pairs,0.01)==pytest.approx((1.,1.))


def test_unconsumed_steps_and_previous_snapshots_are_not_reused():
    from gripper_runtime.physical_contacts import PhysicalContactObserver
    observer=PhysicalContactObserver.__new__(PhysicalContactObserver)
    observer._pending=[contact('left',(0.1,0.,0.))]
    observer._pending_steps=2
    observer.snapshot=[]
    observer.advance()
    assert observer.snapshot is None
    observer.advance()
    assert observer.snapshot==[]
