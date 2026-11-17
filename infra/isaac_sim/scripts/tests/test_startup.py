"""Preserve the runtime startup order while Scene selection is independent."""
import asyncio
from pathlib import Path
import runpy
import sys
from types import ModuleType, SimpleNamespace

import pytest


SCRIPTS = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('headless', [False, True])
def test_startup_keeps_before_play_play_after_play_order(monkeypatch, headless):
    events = []

    async def update():
        events.append('update')

    timeline = SimpleNamespace(play=lambda: events.append('play'), is_playing=lambda: True)
    omni = ModuleType('omni')
    omni.kit = ModuleType('omni.kit')
    omni.kit.app = ModuleType('omni.kit.app')
    omni.kit.app.get_app = lambda: SimpleNamespace(next_update_async=update)
    omni.timeline = ModuleType('omni.timeline')
    omni.timeline.get_timeline_interface = lambda: timeline
    for name, module in [('omni', omni), ('omni.kit', omni.kit),
                         ('omni.kit.app', omni.kit.app), ('omni.timeline', omni.timeline)]:
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setitem(sys.modules, 'carb', SimpleNamespace(log_info=lambda _: None,
                                                          log_error=lambda error: pytest.fail(error)))
    monkeypatch.setenv('SFTWIN_PROJECT_ROOT', str(SCRIPTS.parents[2]))
    monkeypatch.setenv('SFTWIN_HEADLESS', '1' if headless else '0')
    from isaac_runtime import runtime_workflow
    monkeypatch.setattr(runtime_workflow, 'prepare_startup_model',
                        lambda *_: events.append('prepare'))
    loop = asyncio.new_event_loop()
    monkeypatch.setattr(asyncio, 'ensure_future', loop.create_task)
    try:
        namespace = runpy.run_path(str(SCRIPTS / 'isaac_arm_cell_startup.py'))
        start = namespace['_start_arm_cell']
        # Execute the real async startup against fake Kit/Timeline services.
        monkeypatch.setitem(start.__globals__, '_run_entrypoint', lambda path: events.append((path.parent, path.name)))
        loop.run_until_complete(namespace['_ARM_CELL_STARTUP_TASK'])
    finally:
        loop.close()
    assert events == ['prepare', 'update', (SCRIPTS, '1_before_play.py'), 'play', 'update',
                      (SCRIPTS, '2_after_play_headless.py' if headless else '2_after_play.py')]


@pytest.mark.parametrize('initialized',[False,True])
def test_missing_sdk_context_is_initialized_before_play(monkeypatch,initialized):
    events=[]
    class World:
        ready=initialized
        def get_physics_context(self):return object() if self.ready else None
        async def initialize_simulation_context_async(self):
            events.append('physics_ready');self.ready=True
    world=World()
    async def update():pass
    def play():
        assert world.ready
        events.append('play')
    timeline=SimpleNamespace(play=play,is_playing=lambda:True)
    omni=ModuleType('omni');omni.kit=ModuleType('omni.kit');omni.kit.app=ModuleType('omni.kit.app')
    omni.kit.app.get_app=lambda:SimpleNamespace(next_update_async=update)
    omni.timeline=ModuleType('omni.timeline');omni.timeline.get_timeline_interface=lambda:timeline
    for name,module in [('omni',omni),('omni.kit',omni.kit),('omni.kit.app',omni.kit.app),('omni.timeline',omni.timeline)]:monkeypatch.setitem(sys.modules,name,module)
    monkeypatch.setitem(sys.modules,'carb',SimpleNamespace(log_info=lambda _:None,log_error=lambda e:pytest.fail(e)))
    monkeypatch.setenv('SFTWIN_PROJECT_ROOT',str(SCRIPTS.parents[2]))
    from isaac_runtime import runtime_workflow
    monkeypatch.setattr(runtime_workflow,'startup_namespace',{'context':{'world':world}},raising=False)
    monkeypatch.setattr(runtime_workflow,'prepare_startup_model',lambda *_:None)
    loop=asyncio.new_event_loop();monkeypatch.setattr(asyncio,'ensure_future',loop.create_task)
    try:
        namespace=runpy.run_path(str(SCRIPTS/'isaac_arm_cell_startup.py'))
        monkeypatch.setitem(namespace['_start_arm_cell'].__globals__,'_run_entrypoint',lambda _:None)
        loop.run_until_complete(namespace['_ARM_CELL_STARTUP_TASK'])
    finally:loop.close()
    assert events==(['play'] if initialized else ['physics_ready','play'])
