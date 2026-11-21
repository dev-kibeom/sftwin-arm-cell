"""Run the ARM Cell Kit startup sequence after the Isaac app is ready."""

import asyncio
import atexit
import os
import sys
from pathlib import Path

import carb
import omni.kit.app
import omni.timeline


PROJECT_ROOT = Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()
ENTRYPOINTS = PROJECT_ROOT / "infra/isaac_sim/scripts"
BEFORE_PLAY = ENTRYPOINTS / "1_before_play.py"
HEADLESS = os.environ.get("SFTWIN_HEADLESS") == "1"
AFTER_PLAY = ENTRYPOINTS / (
    "2_after_play_headless.py" if HEADLESS else "2_after_play.py"
)

if str(ENTRYPOINTS) not in sys.path:
    sys.path.insert(0, str(ENTRYPOINTS))
from isaac_runtime import runtime_workflow
# Kit re-execution keeps cleanup handles in one namespace, just like Script Editor.
_RUNTIME_NAMESPACE = getattr(runtime_workflow, "startup_namespace", None)
if _RUNTIME_NAMESPACE is None:
    _RUNTIME_NAMESPACE = {"__name__": "__main__"}
    runtime_workflow.startup_namespace = _RUNTIME_NAMESPACE


def _run_entrypoint(path):
    if not path.is_file():
        raise FileNotFoundError(f"ARM Cell startup entrypoint is missing: {path}")
    source = compile(path.read_text(encoding="utf-8"), str(path), "exec")
    _RUNTIME_NAMESPACE["__file__"] = str(path)
    exec(source, _RUNTIME_NAMESPACE)


async def _start_arm_cell():
    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()

    # Let Kit finish creating the initial stage before authoring the cell.
    await app.next_update_async()
    carb.log_info("ARM Cell startup: running before-play setup")
    _run_entrypoint(BEFORE_PLAY)

    # World creation from a Kit extension is asynchronous. Complete the SDK
    # context before Play so later initialization cannot invalidate live views.
    # Scene builders return Cell-owned contexts with different representations.
    # Before-play setup imports the SDK World used by every selected builder.
    world = _RUNTIME_NAMESPACE["World"].instance()
    if world is not None and world.get_physics_context() is None:
        await world.initialize_simulation_context_async()

    carb.log_info("ARM Cell startup: starting Timeline Play")
    timeline.play()
    await app.next_update_async()
    if not timeline.is_playing():
        raise RuntimeError("Isaac Timeline did not enter Playing state")

    after_play_mode = "headless physics" if HEADLESS else "GUI"
    carb.log_info(f"ARM Cell startup: running {after_play_mode} after-play setup")
    _run_entrypoint(AFTER_PLAY)
    carb.log_info("ARM Cell startup sequence completed")


def _report_startup_failure(task):
    try:
        task.result()
    except Exception as error:
        carb.log_error(f"ARM Cell startup failed: {error!r}")


async def _clear_session(app):
    omni.timeline.get_timeline_interface().stop()
    await app.next_update_async()
    close = _RUNTIME_NAMESPACE.get("close_runtime")
    if close is not None:
        close()
        atexit.unregister(close)
    world = _RUNTIME_NAMESPACE.get("World")
    if world is not None and world.instance() is not None:
        world.clear_instance()
    _RUNTIME_NAMESPACE.clear()
    _RUNTIME_NAMESPACE["__name__"] = "__main__"


async def _initialize_scene(attempt):
    """Retry transient Kit/scene failures in the same open Kit and PLC transition."""
    from isaacsim.core.utils.stage import create_new_stage_async

    app = omni.kit.app.get_app()
    while attempt.exists() and not (attempt / "error").exists():
        try:
            await _clear_session(app)
            (attempt / "stopped").touch()
            while not (attempt / "rebuild").exists():
                if (attempt / "error").exists() or not attempt.exists():
                    return
                await app.next_update_async()
            await create_new_stage_async()
            await _start_arm_cell()
            (attempt / "ready").touch()
            return
        except (OSError, RuntimeError) as error:
            carb.log_warn(f"ARM Cell Initialize: retrying scene setup: {error}")
            await asyncio.sleep(1)


runtime_workflow.initialize_scene = _initialize_scene

# The synchronous importer may pump Kit updates while enabling extensions.
# Run it before entering an asyncio task to avoid reentrant Task execution.
carb.log_info("ARM Cell startup: preparing selected model assets")
runtime_workflow.prepare_startup_model(_RUNTIME_NAMESPACE, PROJECT_ROOT)

_ARM_CELL_STARTUP_TASK = asyncio.ensure_future(_start_arm_cell())
_ARM_CELL_STARTUP_TASK.add_done_callback(_report_startup_failure)
