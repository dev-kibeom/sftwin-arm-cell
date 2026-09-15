"""Explicit M0609 Script Editor environment setup.

For Script Editor execution, set ``PROJECT_ROOT`` below to the absolute SFTwin
repository path, then run this script once.  An already configured
``SFTWIN_PROJECT_ROOT`` is also accepted.  No repository auto-discovery occurs.
"""

import os
from pathlib import Path
import sys


# Script Editor operators may edit this value before running the temporary copy.
# Keep None to require an already configured SFTWIN_PROJECT_ROOT instead.
PROJECT_ROOT = None
CELL_RELATIVE_PATH = Path("infra/isaac_sim/scripts/m0609_cell")


def validate_project_root(project_root):
    """Validate an explicitly supplied SFTwin repository root."""
    root = Path(project_root).expanduser()
    if (
        not root.is_dir()
        or not (root / CELL_RELATIVE_PATH / "shared").is_dir()
        or not (root / "ros2_ws/src/arm_cell").is_dir()
    ):
        raise RuntimeError(f"project root is not a validated SFTwin repository: {root}")
    return root.resolve()


def resolve_project_root(project_root=None, environ=None):
    """Require an explicit root argument or an existing configured root."""
    if project_root:
        return validate_project_root(project_root), "explicit project root"
    environment = os.environ if environ is None else environ
    configured_root = environment.get("SFTWIN_PROJECT_ROOT")
    if configured_root:
        return validate_project_root(configured_root), "existing SFTWIN_PROJECT_ROOT"
    raise RuntimeError(
        "No project root was supplied. Set PROJECT_ROOT in 0_setup_sftwin_env.py "
        "or configure SFTWIN_PROJECT_ROOT before running it."
    )


def configure_environment(project_root=None, environ=None):
    """Validate and set root state for the current Isaac Python process."""
    environment = os.environ if environ is None else environ
    root, source = resolve_project_root(project_root, environment)
    module_root = root / CELL_RELATIVE_PATH
    environment["SFTWIN_PROJECT_ROOT"] = str(root)
    if str(module_root) not in sys.path:
        sys.path.insert(0, str(module_root))
    return {
        "status": "VERIFIED",
        "SFTWIN_PROJECT_ROOT": str(root),
        "module_root": str(module_root),
        "resolution_source": source,
    }


if __name__ == "__main__":
    print(configure_environment(PROJECT_ROOT))
