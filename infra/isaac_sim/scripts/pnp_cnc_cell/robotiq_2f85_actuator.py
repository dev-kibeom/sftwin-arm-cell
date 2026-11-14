"""Compatibility import for the scripted Robotiq runtime actuator."""

import sys
from pathlib import Path

scripts = str(Path(__file__).resolve().parent.parent)
if scripts not in sys.path:
    sys.path.insert(0, scripts)

from gripper_runtime.robotiq_actuator import GripperState, Robotiq2F85Actuator

__all__ = ["GripperState", "Robotiq2F85Actuator"]
