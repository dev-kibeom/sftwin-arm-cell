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
