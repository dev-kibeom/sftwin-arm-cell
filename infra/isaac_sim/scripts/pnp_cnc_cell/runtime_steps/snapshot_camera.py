"""Export approved camera TF from the currently composed USD stage."""

import importlib
import os
from pathlib import Path
import sys

module_root = (
    Path(os.environ["SFTWIN_PROJECT_ROOT"]) / "infra/isaac_sim/scripts/pnp_cnc_cell"
)
if str(module_root) not in sys.path:
    sys.path.insert(0, str(module_root))
import shared.run_records as run_records
import camera_tooling.stage_inspector as stage_inspector

importlib.reload(run_records)
importlib.reload(stage_inspector)
from isaac_runtime.runtime_configuration import load_runtime_configuration
if "runtime_configuration" not in globals():
    runtime_configuration = load_runtime_configuration()

output = Path(runtime_configuration["cell"]["camera"]["snapshot"])
output.parent.mkdir(parents=True, exist_ok=True)
stage_inspector.inspect_stage(
    str(output), stage_inspector.script_editor_reference_prims(os.environ),
    camera_configuration=runtime_configuration["cell"]["camera"],
)
