"""Expose shared configuration resolution to Isaac's source-only interpreter."""
import os
from pathlib import Path
import sys

root = Path(os.environ.get('SFTWIN_PROJECT_ROOT', Path(__file__).resolve().parents[4])).expanduser().resolve()
sys.path.insert(0, str(root / 'ros2_ws/src/configuration'))
from arm_cell_configuration.runtime import load_profile, selected_profile  # noqa: E402,F401
