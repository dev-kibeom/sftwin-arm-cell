"""Observed motor obstruction plus selected grasp width is an estimate, not truth."""
from types import SimpleNamespace
import pytest
from gripper_runtime.grasp_attachment import HoldingState
from gripper_runtime.physical_grasp import PhysicalGraspManager
from gripper_runtime.tool_plugin import ParallelGripperCapture
from test_physics_tool import actuator


def manager():
    gripper, drive = actuator()
    tool = SimpleNamespace(cancel_motion=gripper.cancel_motion,
                           capture=ParallelGripperCapture(max_width_mm=85.))
    result = PhysicalGraspManager(gripper, tool=tool, holding_settings={
        'width_tolerance_mm': 3., 'stopped_dwell_s': 0.2,
        'held_position_tolerance_rad': 0.015})
    return result, gripper, drive


def reference(width=42.5, goal='pick'):
    return dict(motion_goal_uuid=goal, frame_id='base_link',
                values=(0., 0., 0., 1., 0., 0., 0., width, 2.))


def close(manager, gripper):
    assert manager.configure_selected_grasp_reference(reference())
    gripper.close()
    assert manager.arm_close_transaction('pick', 1)


def tick(manager, gripper, dt=0.05):
    gripper.update(dt)
    manager.update(dt)
    return manager.holding_observation().state


def test_sustained_stop_at_expected_width_estimates_held_without_attachment():
    result, gripper, drive = manager()
    close(result, gripper)
    drive.pos = 0.4
    assert tick(result, gripper) is HoldingState.UNKNOWN
    for _ in range(8):
        tick(result, gripper)
    held = result.holding_observation()
    assert held.state is HoldingState.HELD
    assert result.attach_joint_path is None
    assert result.attachment_owned() is False
    assert result.is_grasped
    assert drive.target == pytest.approx(0.8)
    tick(result, gripper)
    assert result.holding_observation().sequence == held.sequence
    assert result.holding_observation().observed_at >= held.observed_at


@pytest.mark.parametrize('position', [0., 0.1, 0.7, 0.8])
def test_empty_close_limits_or_wrong_aperture_do_not_confirm_held(position):
    result, gripper, drive = manager()
    close(result, gripper)
    drive.pos = position
    for _ in range(12):
        tick(result, gripper)
    assert result.holding_observation().state is HoldingState.UNKNOWN


def test_transient_stop_and_invalid_or_replaced_reference_drop_evidence():
    result, gripper, drive = manager()
    close(result, gripper)
    drive.pos = 0.4
    tick(result, gripper)
    drive.vel = 0.1
    assert tick(result, gripper) is HoldingState.UNKNOWN
    drive.vel = 0.
    for _ in range(10):
        tick(result, gripper)
    assert result.is_grasped
    assert not result.configure_selected_grasp_reference({'motion_goal_uuid':'pick','invalidate':True})
    assert tick(result, gripper) is HoldingState.UNKNOWN
    result.configure_selected_grasp_reference(reference(goal='next'))
    for _ in range(12):
        tick(result, gripper)
    assert not result.is_grasped  # Old close cannot consume a new reference.


@pytest.mark.parametrize('packet', [None, {}, reference(float('nan')), reference(100.),
                                  dict(reference(), frame_id=''), dict(reference(), values=[1.])])
def test_missing_or_invalid_reference_never_confirms_held(packet):
    result, gripper, drive = manager()
    assert not result.configure_selected_grasp_reference(packet)
    gripper.close()
    assert not result.arm_close_transaction('pick', 1)
    for _ in range(10):
        tick(result, gripper)
    assert not result.is_grasped


def test_held_motion_loss_open_stop_reset_shutdown():
    result, gripper, drive = manager()
    close(result, gripper)
    drive.pos = 0.4
    for _ in range(10):
        tick(result, gripper)
    assert result.is_grasped
    drive.vel = 0.1
    assert tick(result, gripper) is HoldingState.UNKNOWN
    drive.vel, drive.pos = 0., 0.43
    assert tick(result, gripper) is HoldingState.UNKNOWN
    gripper.open()
    result.note_open_release_command_applied(2, 85.)
    assert tick(result, gripper) is HoldingState.UNKNOWN
    drive.pos = 0.
    assert tick(result, gripper) is HoldingState.RELEASED
    result.cancel_tool_transaction()
    assert tick(result, gripper) is HoldingState.UNKNOWN
    assert result.reset_for_simulation_stop()
    assert result.shutdown()
    assert result.holding_observation().state is HoldingState.UNKNOWN


def test_accepted_close_evidence_survives_pending_reference_retirement_only():
    result, gripper, drive = manager()
    close(result, gripper)
    drive.pos = 0.4
    for _ in range(10):
        tick(result, gripper)
    assert result.is_grasped
    result.configure_selected_grasp_reference(dict(motion_goal_uuid='pick', invalidate=True, retire_pending=True))
    assert result.capture_reference_id is None
    assert tick(result, gripper) is HoldingState.HELD
    # Retirement cannot authorize replayed/new CLOSE using stale selection.
    assert not result.arm_close_transaction('pick', 2)
    assert tick(result, gripper) is HoldingState.UNKNOWN
    close(result, gripper)
    for _ in range(10):
        tick(result, gripper)
    result.configure_selected_grasp_reference(dict(motion_goal_uuid='pick', invalidate=True, retire_pending=True))
    result.configure_selected_grasp_reference(dict(motion_goal_uuid='pick', invalidate=True))
    assert tick(result, gripper) is HoldingState.UNKNOWN


class IntentAdapter:
    def __init__(self, **services):
        self.tool = services['tool']
        self.pending = None
        self.packet = None
        self.status = []
        self.last_intent = None
        self.last_command_generation = 0
        self.last_command_width_mm = None
        self.last_command_is_closing = False

    def read_capture_reference(self):
        packet, self.packet = self.packet, None
        return packet

    def mark_capture_reference_applied(self, identity):
        self.applied = identity

    def read_close_execution_id(self):
        return 'pick'

    def update(self):
        from gripper_runtime.tool_plugin import ToolIntent
        if self.pending is None:
            return False
        self.last_intent, self.pending = ToolIntent(self.pending), None
        self.tool.command(self.last_intent)
        self.last_command_generation += 1
        self.last_command_is_closing = self.last_intent is ToolIntent.GRASP
        self.last_command_width_mm = 85. if self.last_intent is ToolIntent.RELEASE else None
        return True

    def publish_status(self, **status):
        self.status.append(status)


def runtime_session(initial_sequence=0):
    from gripper_runtime.session import GripperRuntimeSession
    from gripper_runtime.tool_plugin import parallel_tool_runtime
    gripper, drive = actuator()
    tool = parallel_tool_runtime(gripper, {'runtime':dict(
        capabilities=['width'], max_width_mm=85., move_duration_s=0.4, supported_shapes=['box'])})
    tool.grasp_factory = lambda **services: PhysicalGraspManager(**services, holding_settings=dict(
        width_tolerance_mm=3., stopped_dwell_s=0.2, held_position_tolerance_rad=0.015))
    result = GripperRuntimeSession(SimpleNamespace(), tool=tool, adapter_cls=IntentAdapter,
        initial_status_sequence=initial_sequence,
        diagnostic_writer=SimpleNamespace(submit=lambda record: True))
    return result, drive


def test_real_session_rejection_held_without_attachment_open_and_restart():
    session, drive = runtime_session()
    adapter = session.sim_adapter
    adapter.pending = 'GRASP'  # Rejected without a selected reference, still publishes UNKNOWN.
    session.on_physics_step(0.05)
    assert adapter.status[-1]['runtime_ready']
    assert not adapter.status[-1]['grasp_confirmed']
    adapter.packet, adapter.pending = reference(), 'GRASP'
    session.on_physics_step(0.05)
    drive.pos = 0.4
    for _ in range(10):
        session.on_physics_step(0.05)
    assert adapter.status[-1]['grasp_confirmed']
    assert not adapter.status[-1]['attached']
    held_sequence = adapter.status[-1]['sequence']
    adapter.packet = dict(motion_goal_uuid='pick', invalidate=True, retire_pending=True)
    session.on_physics_step(0.05)
    assert adapter.status[-1]['grasp_confirmed']
    adapter.pending = 'STOP'
    session.on_physics_step(0.05)
    assert not adapter.status[-1]['grasp_confirmed']
    assert adapter.status[-1]['sequence'] > held_sequence
    adapter.pending = 'RELEASE'
    session.on_physics_step(0.05)
    assert not adapter.status[-1]['released']
    drive.pos = 0.
    session.on_physics_step(0.05)
    assert adapter.status[-1]['released']
    sequence = adapter.status[-1]['sequence']
    assert session.reset_for_simulation_stop()
    assert session.shutdown()
    replacement, _ = runtime_session(sequence)
    replacement.on_physics_step(0.05)
    assert replacement.sim_adapter.status[-1]['sequence'] >= sequence
    assert not replacement.sim_adapter.status[-1]['grasp_confirmed']
    assert replacement.shutdown()


def test_adapter_distinguishes_retirement_from_invalidation_and_rejects_bad_reference():
    import json
    from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
    from test_gripper_runtime import _GenerationController, _Attribute, _Gripper
    controller = _GenerationController(_Attribute(-1.), _Attribute(1), _Attribute(''))
    adapter = SimGripperAdapter(_Gripper(), graph_path='/Graph', controller=controller)
    retirement = dict(motion_goal_uuid='pick', invalidate=True, retire_pending=True)
    controller.capture_reference_attribute.value = json.dumps(retirement)
    assert adapter.read_capture_reference() == retirement
    invalidation = dict(motion_goal_uuid='pick', invalidate=True)
    controller.capture_reference_attribute.value = json.dumps(invalidation)
    assert adapter.read_capture_reference() == invalidation
    assert adapter.read_capture_reference() is None
    adapter.mark_capture_reference_applied('next')
    controller.capture_reference_attribute.value = '{invalid json'
    assert adapter.read_capture_reference() == dict(motion_goal_uuid='next', invalidate=True)


@pytest.mark.parametrize('malformed', ['{invalid', '{"motion_goal_uuid":"next","position":[]}'])
def test_retired_held_loses_evidence_on_malformed_replacement_through_real_adapter(malformed):
    import json
    from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
    from test_gripper_runtime import _GenerationController, _Attribute, _Gripper
    result, gripper, drive = manager()
    close(result, gripper)
    drive.pos = 0.4
    for _ in range(10):
        tick(result, gripper)
    controller = _GenerationController(_Attribute(-1.), _Attribute(1), _Attribute(''))
    adapter = SimGripperAdapter(_Gripper(), graph_path='/Graph', controller=controller)
    adapter.mark_capture_reference_applied('pick')
    controller.capture_reference_attribute.value = json.dumps(dict(
        motion_goal_uuid='pick', invalidate=True, retire_pending=True))
    result.configure_selected_grasp_reference(adapter.read_capture_reference())
    adapter.mark_capture_reference_applied(result.capture_reference_id)
    assert tick(result, gripper) is HoldingState.HELD
    controller.capture_reference_attribute.value = malformed
    result.configure_selected_grasp_reference(adapter.read_capture_reference())
    assert tick(result, gripper) is HoldingState.UNKNOWN


def test_repeated_reference_and_retirement_preserve_inference_identity_until_real_invalidation():
    result, gripper, drive = manager()
    close(result, gripper)
    drive.pos = 0.4
    for _ in range(10):
        tick(result, gripper)
    sequence = result.holding_observation().sequence
    result.configure_selected_grasp_reference(reference())
    assert tick(result, gripper) is HoldingState.HELD
    result.configure_selected_grasp_reference(dict(motion_goal_uuid='other', invalidate=True))
    assert tick(result, gripper) is HoldingState.HELD
    for _ in range(2):
        result.configure_selected_grasp_reference(dict(motion_goal_uuid='pick', invalidate=True, retire_pending=True))
        assert tick(result, gripper) is HoldingState.HELD
    assert result.holding_observation().sequence == sequence
    assert result.diagnostics()['holding_inference']['execution_id'] == 'pick'
    drive.vel = 0.1
    assert tick(result, gripper) is HoldingState.UNKNOWN
