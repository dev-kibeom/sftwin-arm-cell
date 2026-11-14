"""Compatibility entrypoint; execution lives in the common Isaac workflow."""
import os
from pathlib import Path
PROJECT_ROOT = "~/arm_cell"
root_value = PROJECT_ROOT or os.environ.get("SFTWIN_PROJECT_ROOT")
if not root_value:
    raise RuntimeError("Set PROJECT_ROOT or SFTWIN_PROJECT_ROOT before 0_pre_build.py")
entrypoint = Path(root_value).expanduser().resolve() / "infra/isaac_sim/scripts/0_pre_build.py"
exec(compile(entrypoint.read_text(encoding="utf-8"), str(entrypoint), "exec"), globals())
