"""Runtime role ordering and selected implementation isolation."""
from pathlib import Path
import sys

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))


def test_roles_preserve_order_and_one_configuration(monkeypatch):
    from isaac_runtime import runtime_workflow as workflow
    selected = {'selected': object()}
    reads = []
    monkeypatch.setattr(workflow, 'load_runtime_configuration', lambda: reads.append(1) or selected)
    events = []
    def step(filename, namespace):
        events.append(filename)
        assert namespace['runtime_configuration'] is selected
    monkeypatch.setattr(workflow, 'run_step', step)
    namespace = {}
    workflow.run_role('pre_build', namespace)
    workflow.run_role('before_play', namespace)
    workflow.run_role('after_play', namespace)
    assert events == ['build_robot_usd.py', 'setup_scene.py', 'snapshot_camera.py',
                      'build_actiongraph.py', 'bootstrap_tool.py', 'enable_simulation_ui_hub.py']
    assert reads == [1, 1]  # Pre Build is independent; Before/After share a selection.
    events.clear()
    workflow.run_role('after_play_headless', namespace)
    assert events == ['bootstrap_tool.py']
    assert reads == [1, 1]


def test_alternative_tool_does_not_load_m0609_bootstrap(tmp_path, monkeypatch):
    from isaac_runtime import runtime_workflow as workflow
    plugin = tmp_path / 'alternative_tool.py'
    plugin.write_text('def start(*, configuration, namespace):\n'
                      '    namespace["selected_tool"] = configuration["tool"]["name"]\n')
    monkeypatch.syspath_prepend(str(tmp_path))
    configuration = {'tool': {'name': 'alternative', 'runtime': {
        'isaac_bootstrap': 'alternative_tool:start'}}}
    namespace = {'runtime_configuration': configuration}
    workflow.run_step('bootstrap_tool.py', namespace)
    assert namespace['selected_tool'] == 'alternative'
    assert not any(name.endswith('runtime_steps.bootstrap_gripper') for name in sys.modules)


def test_selected_cell_graph_and_camera_share_the_snapshot(tmp_path, monkeypatch):
    from isaac_runtime import runtime_workflow as workflow
    plugin = tmp_path / 'alternative_cell.py'
    plugin.write_text('def graph(*, configuration, namespace):\n'
                      '    namespace["graph_configuration"] = configuration\n'
                      'def camera(*, configuration, namespace):\n'
                      '    namespace["camera_configuration"] = configuration\n')
    monkeypatch.syspath_prepend(str(tmp_path))
    configuration = {'cell': {'isaac': {
        'action_graph_builder': 'alternative_cell:graph',
        'camera_snapshot': 'alternative_cell:camera'}}}
    namespace = {'runtime_configuration': configuration}
    workflow.run_step('snapshot_camera.py', namespace)
    workflow.run_step('build_actiongraph.py', namespace)
    assert namespace['camera_configuration'] is configuration
    assert namespace['graph_configuration'] is configuration


def test_before_play_releases_old_runtime_before_loading_next_configuration(monkeypatch):
    from isaac_runtime import runtime_workflow as workflow
    events = []
    monkeypatch.setattr(workflow, 'load_runtime_configuration', lambda: events.append('load') or {})
    monkeypatch.setattr(workflow, 'run_step', lambda filename, _: events.append(filename))
    workflow.run_role('before_play', {'close_runtime': lambda: events.append('close')})
    assert events == ['close', 'load', 'setup_scene.py', 'snapshot_camera.py', 'build_actiongraph.py']


def test_common_scene_lifecycle_uses_selected_cell_and_preserves_stop_order(tmp_path, monkeypatch):
    """Catch a default-Cell fallback, duplicate callbacks or lost Stop restoration."""
    from types import SimpleNamespace as NS
    from isaac_runtime import runtime_workflow as workflow
    import isaac_runtime.robot_binding as robot_binding
    events = []

    class Subscription:
        def __init__(self, callback):
            self.callback, self.closed = callback, 0
        def unsubscribe(self):
            self.closed += 1
    class Stream:
        def __init__(self):
            self.subscriptions = []
        def create_subscription_to_pop(self, callback, **_):
            sub = Subscription(callback)
            self.subscriptions.append(sub)
            return sub
    updates, timeline_events = Stream(), Stream()
    timeline = NS(is_playing=lambda: False, get_timeline_event_stream=lambda: timeline_events)
    import types
    omni = types.ModuleType('omni')
    kit = types.ModuleType('omni.kit')
    kit.app = NS(get_app=lambda: NS(get_update_event_stream=lambda: updates))
    omni.kit = kit
    omni.timeline = NS(get_timeline_interface=lambda: timeline,
                       TimelineEventType=NS(STOP=2, PLAY=1, PAUSE=3))
    for name, module in [('omni', omni), ('omni.kit', kit), ('omni.kit.app', kit.app),
                         ('omni.timeline', omni.timeline)]:
        monkeypatch.setitem(sys.modules, name, module)
    world = NS(stage=object())
    monkeypatch.setitem(sys.modules, 'isaacsim.core.api', NS(World=NS(instance=lambda: world)))
    for module, name in [('extensions', 'enable_extension'), ('prims', 'delete_prim'),
                         ('stage', 'add_reference_to_stage')]:
        monkeypatch.setitem(sys.modules, f'isaacsim.core.utils.{module}', NS(**{name: lambda *a: None}))
    monkeypatch.setitem(sys.modules, 'pxr', NS(Gf=object(), UsdGeom=object(), UsdLux=object(), UsdPhysics=object()))
    monkeypatch.setattr(robot_binding, 'author_home_targets', lambda *a: events.append('home'))
    monkeypatch.setattr(robot_binding, 'apply_joint_drive_profile', lambda *a: events.append('drive'))
    configuration = {'cell': {'isaac': {'robot_prim_path': '/Selected/Arm',
                                       'material_supply_factory': 'alternative_lifecycle:supply',
                                       'scene_builder': 'alternative_lifecycle:scene'}},
                     'model_configuration': {'isaac': {'articulation_root': 'root', 'articulation_base': 'base'}},
                     'motion': {'arm_joint_names': ['joint']}, 'home_joint_positions': [0], 'robot': {}}
    received = []
    plugin = types.ModuleType('alternative_lifecycle')
    plugin.scene = lambda **kwargs: received.append(kwargs['configuration']) or object()
    adapter = NS(spin_once=lambda: events.append('spin'),
                 reset_generated_materials=lambda **kwargs: events.append(('materials', kwargs['attachment_reset']())) or True,
                 set_live_test_session_id=lambda *a: None, close=lambda: events.append('supply_close'))
    plugin.supply = lambda **kwargs: received.append(kwargs['configuration']) or adapter
    monkeypatch.setitem(sys.modules, 'alternative_lifecycle', plugin)
    monkeypatch.setenv('SFTWIN_PROJECT_ROOT', str(tmp_path))
    namespace = {'runtime_configuration': configuration}
    workflow.run_step('setup_scene.py', namespace)
    assert received == [configuration, configuration]
    assert events == ['home', 'drive']
    namespace['runtime_session'] = NS(
        reset_for_simulation_stop=lambda: events.append('tool_reset') or True,
        close_diagnostic_writer=lambda: events.append('writer_close'),
        shutdown=lambda: events.append('tool_shutdown'))
    event = timeline_events.subscriptions[0].callback
    event(NS(type=1))
    first = namespace['_live_test_session']
    assert first is not None
    event(NS(type=3))  # Pause must preserve the live Session.
    event(NS(type=1))
    assert namespace['_live_test_session'] is first
    events.clear()
    event(NS(type=2))
    assert events == ['tool_reset', 'writer_close', 'home', ('materials', True)]
    assert namespace['_live_test_session'] is None
    event(NS(type=1))
    assert namespace['_live_test_session'].session_id != first.session_id
    assert len(timeline_events.subscriptions) == len(updates.subscriptions) == 1
    namespace['close_runtime']()
    namespace['close_runtime']()
    assert events.count('tool_shutdown') == events.count('supply_close') == 1
    assert updates.subscriptions[0].closed == timeline_events.subscriptions[0].closed == 1
    import atexit
    atexit.unregister(namespace['close_runtime'])
