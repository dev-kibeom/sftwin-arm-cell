"""Tool intent execution must not require a width or joint API."""
from types import SimpleNamespace

import pytest

from gripper_runtime.tool_plugin import ToolIntent, ToolRuntime, select_tool_runtime


class Vacuum:
    def __init__(self):
        self.enabled = False

    def vacuum_on(self):
        self.enabled = True

    def vacuum_off(self):
        self.enabled = False


def test_widthless_intents_are_translated_by_selected_tool():
    vacuum = Vacuum()
    tool = ToolRuntime(
        actuator=vacuum, capabilities=frozenset({'vacuum'}),
        execute=lambda intent, duration: (
            vacuum.vacuum_on() if intent is ToolIntent.GRASP else vacuum.vacuum_off()
        ),
        capture=SimpleNamespace(),
    )
    tool.command(ToolIntent.GRASP)
    assert vacuum.enabled
    assert tool.width_mm() is None
    tool.command(ToolIntent.RELEASE)
    assert not vacuum.enabled


def test_profile_selects_implementation_and_passes_assembly_binding():
    binding = SimpleNamespace(robot_path='/Robot', articulation_root_path='/Robot/root', base_link_path='/Robot/base')
    profile = {'runtime': {'implementation': 'test-vacuum', 'capabilities': ['vacuum']}}
    selected = select_tool_runtime(profile, binding, implementations={
        'test-vacuum': lambda profile, binding: (profile, binding)
    })
    assert selected == (profile, binding)


def test_widthless_tool_uses_real_session_capture_and_release_ownership():
    from dataclasses import replace
    from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
    from gripper_runtime.grasp_attachment import GraspManager, HoldingState
    from gripper_runtime.grasp_policy import CaptureDecision, CaptureResult
    from gripper_runtime.grasp_qualification import GraspQualificationResult
    from gripper_runtime.session import GripperRuntimeSession
    from test_grasp_attachment import FakeCandidateSource, FakeJointOwner, valid_candidate, capture_config
    import json

    class VacuumActuator(Vacuum):
        def __init__(self):
            super().__init__()
            self.state = SimpleNamespace(value='holding')
            self.complete = False

        def update(self, dt):
            if self.complete:
                self.state.value = 'holding'

    class VacuumCapture:
        def decode_reference(self, packet):
            # This Tool uses a seal surface, with no parallel contact width.
            return packet if packet.get('seal_surface') == 'top' else None

        def configure_selected_reference(self, source, packet):
            source.seal_surface = packet['seal_surface']
            return True

        def on_capture_ready(self, actuator):
            pass  # A vacuum must remain on while the shared manager attaches.

        def select(self, **inputs):
            candidates = inputs['candidates']
            if not sealed['value']:
                return CaptureResult(CaptureDecision.NO_CAPTURE)
            return CaptureResult(CaptureDecision.CAPTURE_ELIGIBLE, candidates[0])

        def qualify(self, candidate):
            return GraspQualificationResult(vacuum.enabled and sealed['value'], 'seal_missing')

        def ready_during_grasp(self, candidate, source):
            return sealed['value']

        def release_complete(self, actuator, target):
            assert target is None
            return not actuator.enabled and actuator.complete

    class Attribute:
        def __init__(self, value):
            self.value = value

        def is_valid(self):
            return True

    class Graph:
        def __init__(self):
            self.values = {
                'ToolIntentGeneration.outputs:count': Attribute(0),
                'GripperCommandGeneration.outputs:count': Attribute(0),
                'GripperCloseGeneration.outputs:count': Attribute(0),
                'SubscribeGripperClose.outputs:data': Attribute('vacuum-pick'),
                'SubscribeToolIntent.outputs:data': Attribute(''),
                'SubscribeGripperCaptureReference.outputs:data': Attribute(json.dumps({
                    'motion_goal_uuid': 'vacuum-pick', 'seal_surface': 'top'})),
                'PublishGripperStatus.inputs:data': Attribute(''),
            }

        def attribute(self, path):
            return self.values.get(path.rsplit('/', 1)[-1])

        def get(self, attr):
            return attr.value

        def set(self, attr, value):
            attr.value = value

        def command(self, intent, execution_id="vacuum-pick"):
            self.values['ToolIntentGeneration.outputs:count'].value += 1
            self.values['SubscribeToolIntent.outputs:data'].value = json.dumps({'intent': intent, 'execution_id': execution_id})

    vacuum, graph, joints = VacuumActuator(), Graph(), FakeJointOwner()
    sealed = {'value': False}
    source = FakeCandidateSource([replace(valid_candidate(), measured_opening_mm=None, capture_reference=None)])
    def execute(intent, duration):
        if intent is ToolIntent.GRASP:
            vacuum.vacuum_on()
        elif intent is ToolIntent.RELEASE:
            vacuum.vacuum_off()
        else:
            vacuum.state.value = 'holding'
            vacuum.complete = False

    tool = ToolRuntime(vacuum, frozenset({'vacuum'}), execute, VacuumCapture())
    session = GripperRuntimeSession(
        SimpleNamespace(), tool=tool,
        adapter_cls=lambda **kwargs: SimGripperAdapter(controller=graph, **kwargs),
        grasp_manager_cls=lambda **kwargs: GraspManager(tcp_path='/TCP', candidate_source=source, joint_owner=joints, **kwargs),
        capture_config=capture_config(), diagnostic_writer=SimpleNamespace(submit=lambda record: True),
    )
    session.on_physics_step(0.01)
    released = session.grasp_manager.holding_observation()
    assert released.state is HoldingState.UNKNOWN  # no completed physical release yet
    assert source.seal_surface == 'top'
    graph.command('GRASP')
    session.on_physics_step(0.01)
    assert vacuum.enabled
    assert not joints.has_owned_joint()
    assert session.grasp_manager.holding_observation().state is HoldingState.UNKNOWN
    # A new grasp with a verified seal is qualified by the Tool, not the parallel policy.
    sealed['value'] = True
    graph.command('GRASP')
    session.on_physics_step(0.01)
    held = session.grasp_manager.holding_observation()
    assert held.state is HoldingState.HELD
    assert joints.has_owned_joint()
    graph.command('RELEASE')
    vacuum.state.value = 'opening'
    session.on_physics_step(0.01)
    assert not vacuum.enabled
    assert joints.has_owned_joint()  # drive command alone cannot detach
    assert session.grasp_manager.holding_observation().sequence == held.sequence
    vacuum.complete = True
    session.on_physics_step(0.01)
    released = session.grasp_manager.holding_observation()
    assert released.state is HoldingState.RELEASED
    assert released.sequence > held.sequence
    assert not joints.has_owned_joint()
    payload = graph.values['PublishGripperStatus.inputs:data'].value
    assert 'released=1' in payload
    assert 'width_mm=' not in payload
    # A pending grasp must not attach after an explicit Tool STOP.
    graph.values['SubscribeGripperCaptureReference.outputs:data'].value = json.dumps({
        'motion_goal_uuid': 'vacuum-pick', 'seal_surface': 'top', 'revision': 2})
    sealed['value'] = False
    vacuum.complete = False
    vacuum.state.value = 'closing'
    graph.command('GRASP')
    session.on_physics_step(0.01)
    # A legacy close received before STOP must not replay on the next tick.
    graph.values['GripperCloseGeneration.outputs:count'].value += 1
    graph.command('STOP')
    session.on_physics_step(0.01)
    sealed['value'] = True
    session.on_physics_step(0.01)
    assert not joints.has_owned_joint()
    assert session.grasp_manager.holding_observation().state is HoldingState.UNKNOWN
    # Rejected reference must never be acknowledged or capture using the old source.
    tool.capture.configure_selected_reference = lambda source, packet: False
    graph.values['SubscribeGripperCaptureReference.outputs:data'].value = json.dumps({
        'motion_goal_uuid': 'rejected-pick', 'seal_surface': 'top'})
    graph.command('GRASP', execution_id='rejected-pick')
    session.on_physics_step(0.01)
    assert not joints.has_owned_joint()
    assert session.sim_adapter._applied_capture_reference_id is None
    assert session.reset_for_simulation_stop()
    assert session.grasp_manager.holding_observation().state is HoldingState.UNKNOWN
    assert session.shutdown()


def test_fake_tool_is_not_a_production_implementation():
    with pytest.raises(ValueError, match='Unsupported Tool runtime'):
        select_tool_runtime({'runtime': {'implementation': 'test-vacuum'}}, None)
