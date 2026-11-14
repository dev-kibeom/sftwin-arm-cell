"""Compatibility module for existing Script Editor tooling."""
import sys
from pathlib import Path
scripts = str(Path(__file__).resolve().parents[2])
if scripts not in sys.path:
    sys.path.insert(0, scripts)
# Alias the module so caches and monkeypatches retain a single owner.
from isaac_runtime import run_records as implementation
sys.modules[__name__] = implementation
