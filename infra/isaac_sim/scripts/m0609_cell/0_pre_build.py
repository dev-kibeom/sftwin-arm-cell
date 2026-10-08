"""Prepare generated robot assets using the configured repository root."""

import importlib
import os
from pathlib import Path
import sys

# Set this only for a deliberate local override; otherwise configure
# SFTWIN_PROJECT_ROOT in the Isaac process.
PROJECT_ROOT = None
root_value = PROJECT_ROOT or os.environ.get("SFTWIN_PROJECT_ROOT")
if not root_value:
    raise RuntimeError(
        "Set PROJECT_ROOT or SFTWIN_PROJECT_ROOT before asset preparation"
    )
root = Path(root_value).expanduser().resolve()
module_root = root / "infra/isaac_sim/scripts/m0609_cell"
if (
    not (module_root / "shared").is_dir()
    or not (root / "ros2_ws/src/arm_cell").is_dir()
):
    raise RuntimeError(f"project root is not a validated SFTwin repository: {root}")
os.environ["SFTWIN_PROJECT_ROOT"] = str(root)
if str(module_root) not in sys.path:
    sys.path.insert(0, str(module_root))

import shared.role_entrypoint as role_entrypoint

importlib.reload(role_entrypoint).run("pre_build", globals())
