"""Run the ARM Cell Kit startup sequence after the Isaac app is ready."""

import asyncio
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


_ARM_CELL_STARTUP_TASK = asyncio.ensure_future(_start_arm_cell())
_ARM_CELL_STARTUP_TASK.add_done_callback(_report_startup_failure)
