"""Configure the production runtime and compose the ARM Cell before Play."""

import importlib
import importlib.util
import os
from pathlib import Path
import sys

# Configure SFTWIN_PROJECT_ROOT in the Isaac process, or set this only for a
# deliberate local override.
PROJECT_ROOT = None
root_value = PROJECT_ROOT or os.environ.get("SFTWIN_PROJECT_ROOT")
if not root_value:
    raise RuntimeError(
        "Set PROJECT_ROOT or SFTWIN_PROJECT_ROOT before 1_before_play.py"
    )
root = Path(root_value).expanduser().resolve()
setup_path = root / "infra/isaac_sim/scripts/m0609_cell/shared/environment_setup.py"
spec = importlib.util.spec_from_file_location("sftwin_environment_setup", setup_path)
if spec is None or spec.loader is None:
    raise RuntimeError(f"cannot load live environment setup from {setup_path}")
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)
setup.configure_live_environment(root)
os.environ["SFTWIN_RUNTIME_MODE"] = "production"

import shared.role_entrypoint as role_entrypoint

importlib.reload(role_entrypoint).run("before_play", globals())
