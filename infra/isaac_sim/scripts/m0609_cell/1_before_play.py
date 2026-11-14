"""Compatibility entrypoint; execution lives in the common Isaac workflow."""
import os
from pathlib import Path
PROJECT_ROOT = None
DIAGNOSTICS_SELECTION = "all"
root_value = PROJECT_ROOT or os.environ.get("SFTWIN_PROJECT_ROOT")
if not root_value:
    raise RuntimeError("Set PROJECT_ROOT or SFTWIN_PROJECT_ROOT before 1_before_play.py")
entrypoint = Path(root_value).expanduser().resolve() / "infra/isaac_sim/scripts/1_before_play.py"
exec(compile(entrypoint.read_text(encoding="utf-8"), str(entrypoint), "exec"), globals())
