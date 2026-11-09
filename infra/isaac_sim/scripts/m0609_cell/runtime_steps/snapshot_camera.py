"""Export approved camera TF from the currently composed USD stage."""

import importlib
import os
from pathlib import Path
import sys

module_root = (
    Path(os.environ["SFTWIN_PROJECT_ROOT"]) / "infra/isaac_sim/scripts/m0609_cell"
)
if str(module_root) not in sys.path:
    sys.path.insert(0, str(module_root))
import shared.local_artifacts as local_artifacts
import camera_tooling.stage_inspector as stage_inspector

importlib.reload(local_artifacts)
importlib.reload(stage_inspector)
output = stage_inspector.script_editor_output_path(os.environ, __file__)
stage_inspector.inspect_stage(
    str(output), stage_inspector.script_editor_reference_prims(os.environ)
)
