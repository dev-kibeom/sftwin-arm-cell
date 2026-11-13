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
    loop = asyncio.new_event_loop()
    monkeypatch.setattr(asyncio, 'ensure_future', loop.create_task)
    try:
        namespace = runpy.run_path(str(SCRIPTS / 'isaac_arm_cell_startup.py'))
        start = namespace['_start_arm_cell']
        # Execute the real async startup against fake Kit/Timeline services.
        monkeypatch.setitem(start.__globals__, '_run_entrypoint', lambda path: events.append(path.name))
        loop.run_until_complete(namespace['_ARM_CELL_STARTUP_TASK'])
    finally:
        loop.close()
    assert events == ['update', '1_before_play.py', 'play', 'update',
                      '2_after_play_headless.py' if headless else '2_after_play.py']
