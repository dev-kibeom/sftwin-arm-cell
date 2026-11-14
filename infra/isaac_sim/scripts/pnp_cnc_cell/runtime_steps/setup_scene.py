"""Compatibility entrypoint for the common Isaac backend step."""
import os
import sys
from pathlib import Path
scripts = str(Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser() / "infra/isaac_sim/scripts")
if scripts not in sys.path:
    sys.path.insert(0, scripts)
from isaac_runtime.runtime_configuration import load_runtime_configuration
from isaac_runtime.runtime_workflow import run_step
if "runtime_configuration" not in globals():
    runtime_configuration = load_runtime_configuration()
run_step("setup_scene.py", globals())
