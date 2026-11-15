"""Mode contracts distinguish detection from attachment ownership."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
from gripper_runtime.grasp_attachment import GraspManager, HoldingState
from gripper_runtime.tool_plugin import ParallelGripperCapture
from test_physics_tool import actuator
from test_physical_holding import manager, close, tick, reference
from test_contact_reaction import contact
from test_grasp_attachment import FakeCandidateSource, FakeJointOwner, valid_candidate, capture_config


def test_real_detection_needs_no_contact_sensor_and_tolerates_velocity_jitter():
    result, gripper, drive = manager()
    close(result, gripper)
    result._observer = None
    for i in range(12):
        drive.pos = 0.4 + (i % 2) * 0.0001
        drive.vel = (-1 if i % 2 else 1) * 0.07
        tick(result, gripper)
    assert result.is_grasped
    assert gripper.object_detection == 'stopped_closing'
    assert not result.attachment_owned()
    drive.pos = 0.44
    assert tick(result, gripper) is HoldingState.UNKNOWN


def snap():
    from gripper_runtime.physical_snap import PhysicalSnapCapture
    gripper, drive = actuator()
    candidate = replace(valid_candidate('/material'), measured_opening_mm=42.5,
                        capture_reference={'expected_contact_width_mm':42.5})
    source = FakeCandidateSource([candidate])
    source.selected_object_path = '/material'
    contacts = SimpleNamespace(snapshot=[], advance=lambda:None, clear=lambda:None, close=lambda:None)
    capture = PhysicalSnapCapture(settings=dict(min_contact_force_per_finger_n=0.1,
        width_tolerance_mm=3., stopped_dwell_s=0.2, held_position_tolerance_rad=0.015),
        observer=contacts, fingertip_paths=('/robot/gripper_robotiq_85_left_finger_tip_link',
                                          '/robot/gripper_robotiq_85_right_finger_tip_link'))
    tool = SimpleNamespace(capture=capture, cancel_motion=gripper.cancel_motion,
                           width_mm=gripper.get_width)
    owner = FakeJointOwner()
    result = GraspManager(gripper, tool=tool, tcp_path='/robot/tcp',
                          candidate_source=source, joint_owner=owner, config=capture_config())
    result.capture_reference_id = 'pick'
    gripper.close()
    assert result.arm_close_transaction('pick', 1)
    drive.pos=0.4
    return result, gripper, drive, contacts, owner


@pytest.mark.parametrize('case', ['one_finger','different_body','wrong_reference','wrong_width','limit','target_reached'])
def test_snap_rejects_unqualified_contact(case):
    result, gripper, drive, contacts, owner = snap()
    contacts.snapshot=[contact('left',(0.1,0.,0.)),contact('right',(-0.1,0.,0.))]
    if case=='one_finger': contacts.snapshot.pop()
    if case=='different_body': contacts.snapshot[1]['actor1']='/tray'
    if case=='wrong_reference': result.candidate_source.selected_object_path='/other'
    if case=='wrong_width': drive.pos=0.6
    if case=='limit': drive.pos=0.
    if case=='target_reached': drive.pos=0.8
    for _ in range(12): tick(result,gripper)
    assert result.holding_observation().state is not HoldingState.HELD
    assert not owner.has_owned_joint()


def test_snap_qualification_waits_then_held_requires_owned_attachment():
    result, gripper, drive, contacts, owner = snap()
    contacts.snapshot=[contact('left',(0.1,0.,0.)),contact('right',(-0.1,0.,0.))]
    tick(result,gripper)
    assert not result.is_grasped and not owner.has_owned_joint()
    for _ in range(12): tick(result,gripper)
    assert owner.has_owned_joint() and result.is_grasped


@pytest.mark.parametrize('failure', ['create', 'validate'])
def test_snap_attach_failure_never_publishes_held(failure):
    result, gripper, drive, contacts, owner = snap()
    contacts.snapshot=[contact('left',(0.1,0.,0.)),contact('right',(-0.1,0.,0.))]
    if failure == 'create': owner.create=lambda target:False
    else: owner.valid=False
    for _ in range(12):
        assert tick(result,gripper) is not HoldingState.HELD


def test_snap_release_and_lifecycle_use_existing_owner():
    result, gripper, drive, contacts, owner = snap()
    contacts.snapshot=[contact('left',(0.1,0.,0.)),contact('right',(-0.1,0.,0.))]
    for _ in range(12): tick(result,gripper)
    assert result.is_grasped
    gripper.open()
    result.note_open_release_command_applied(2,85.)
    tick(result,gripper)
    assert result.holding_observation().state is not HoldingState.RELEASED
    drive.pos=0.005  # Broad joint tolerance alone is not full OPEN.
    tick(result,gripper)
    assert result._pending_release_command is not None
    assert result.holding_observation().state is not HoldingState.RELEASED
    drive.pos=0.
    tick(result,gripper)
    assert not owner.has_owned_joint()
    assert result.holding_observation().state is HoldingState.RELEASED
    result.cancel_tool_transaction()
    assert result.reset_for_simulation_stop()
    assert result.shutdown()
    assert result.holding_observation().state is HoldingState.UNKNOWN


def test_snap_close_reference_and_contact_loss_reset_dwell():
    result, gripper, drive, contacts, owner = snap()
    assert not result.arm_close_transaction('other',2)
    assert result.arm_close_transaction('pick',3)
    contacts.snapshot=[contact('left',(0.1,0.,0.)),contact('right',(-0.1,0.,0.))]
    tick(result,gripper)
    contacts.snapshot=[]
    tick(result,gripper)
    contacts.snapshot=[contact('left',(0.1,0.,0.)),contact('right',(-0.1,0.,0.))]
    tick(result,gripper)
    assert not result.is_grasped
    result.cancel_tool_transaction()
    for _ in range(10):tick(result,gripper)
    assert not owner.has_owned_joint()


def test_real_contact_diagnostic_failure_does_not_change_detection():
    result, gripper, drive = manager()
    close(result,gripper)
    drive.pos=0.4
    def fail(): raise RuntimeError('contact reports unavailable')
    result._observer.advance=fail
    for _ in range(12): tick(result,gripper)
    assert result.is_grasped


@pytest.mark.parametrize('combination,mode', [
    (('logical','fixed_attach'),'logical_snap'),
    (('physics','fixed_attach'),'physical_snap'),
    (('physics','physical'),'physical_real')])
def test_selected_runtime_uses_existing_actuator_and_grasp_boundary(monkeypatch, exported_configuration, combination, mode):
    import copy
    import json
    import sys
    from gripper_runtime import physics_actuator, robotiq_actuator, physical_contacts
    from gripper_runtime.physical_snap import PhysicalSnapCapture
    from gripper_runtime.tool_plugin import select_tool_runtime
    from test_physical_holding import observer
    profile=copy.deepcopy(json.loads(exported_configuration.read_text())['configuration']['tool'])
    profile['runtime']['actuator'],profile['runtime']['grasp']=combination
    profile['runtime']['force_transmission'] = dict(finger_lever_x_m=.0371575, finger_lever_z_m=.04342168)
    profile['runtime']['joint_model'] = {}  # Assembly-independent actuator test double.
    gripper,drive=actuator()
    monkeypatch.setattr(physics_actuator,'Robotiq2F85PhysicsActuator',lambda **kwargs:gripper)
    logical=SimpleNamespace()
    monkeypatch.setattr(robotiq_actuator,'Robotiq2F85Actuator',lambda **kwargs:logical)
    monkeypatch.setattr(physical_contacts,'PhysicalContactObserver',observer)
    prim=SimpleNamespace(GetName=lambda:'sf_grasp_tcp', GetPath=lambda:'/robot/sf_grasp_tcp')
    usd=SimpleNamespace(get_context=lambda:SimpleNamespace(get_stage=lambda:SimpleNamespace(Traverse=lambda:[prim])))
    monkeypatch.setitem(sys.modules,'omni',SimpleNamespace(usd=usd))
    monkeypatch.setitem(sys.modules,'omni.usd',usd)
    tool=select_tool_runtime(profile,SimpleNamespace(robot_path='/robot',articulation_root_path='/robot/root',base_link_path='/robot/base'))
    assert tool.actuator is (logical if mode=='logical_snap' else gripper)
    assert isinstance(tool.capture,PhysicalSnapCapture) == (mode=='physical_snap')
    assert (tool.grasp_factory is not None) == (mode=='physical_real')


def test_snap_bilateral_subthreshold_reaction_cannot_attach():
    result,gripper,drive,contacts,owner=snap()
    contacts.snapshot=[contact('left',(0.0001,0.,0.)),contact('right',(-0.0001,0.,0.))]
    for _ in range(20):tick(result,gripper)
    assert not result.is_grasped and not owner.has_owned_joint()
