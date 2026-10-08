"""Compatibility entrypoint for the M0609 runtime articulation probe."""

import os
from pathlib import Path
import sys


def _add_cell_module_root():
    """Load the shared resolver before delegating all root validation to it."""
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    bootstrap_root = (
        Path(configured_root).expanduser() / "infra/isaac_sim/scripts/m0609_cell"
        if configured_root
        else Path(__file__).resolve().parents[1] / "m0609_cell"
    )
    if not (bootstrap_root / "shared").is_dir():
        raise RuntimeError(
            "Cannot locate M0609 Script Editor support. When running from Isaac "
            "Script Editor, set SFTWIN_PROJECT_ROOT to the repository root."
        )
    if str(bootstrap_root) not in sys.path:
        sys.path.insert(0, str(bootstrap_root))
    from shared.script_editor_bootstrap import add_module_root

    return add_module_root(__file__)


_add_cell_module_root()

from shared.runtime_articulation_probe import run_probe

run_probe()
