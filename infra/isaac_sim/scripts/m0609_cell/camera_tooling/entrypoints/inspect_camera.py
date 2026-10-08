"""Script Editor entrypoint for read-only camera inspection."""

import json
import os
from pathlib import Path
import sys

OUTPUT_FILENAME = "camera_snapshot.json"


def _add_cell_module_root():
    """Load the shared resolver before delegating all root validation to it."""
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if configured_root:
        bootstrap_root = (
            Path(configured_root).expanduser() / "infra/isaac_sim/scripts/m0609_cell"
        )
    else:
        source = Path(__file__).resolve()
        bootstrap_root = next(
            (parent for parent in source.parents if parent.name == "m0609_cell"),
            source.parent,
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


from camera_tooling.stage_inspector import *  # noqa: F401,F403 - entrypoint API

if __name__ == "__main__":
    print(
        json.dumps(
            run_entrypoint(script_path=__file__, output_filename=OUTPUT_FILENAME),
            indent=2,
            default=list,
        )
    )
