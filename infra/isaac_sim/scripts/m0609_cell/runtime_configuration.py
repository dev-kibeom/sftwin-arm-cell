"""Compatibility import for existing Cell tooling."""
import sys
from pathlib import Path
scripts = str(Path(__file__).resolve().parent.parent)
if scripts not in sys.path:
    sys.path.insert(0, scripts)
from isaac_runtime.runtime_configuration import *
