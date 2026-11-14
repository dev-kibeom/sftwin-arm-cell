"""Compatibility import for the ActionGraph gripper command adapter."""

import sys
from pathlib import Path

scripts = str(Path(__file__).resolve().parent.parent)
if scripts not in sys.path:
    sys.path.insert(0, scripts)

from gripper_runtime.action_graph_command_adapter import SimGripperAdapter

__all__ = ["SimGripperAdapter"]
