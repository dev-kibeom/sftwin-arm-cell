"""Pytest path setup for M0609 cell-owned verification code."""

import sys
from pathlib import Path


TEST_ROOT = Path(__file__).resolve().parent
CELL_MODULE_ROOT = TEST_ROOT.parent
if str(CELL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CELL_MODULE_ROOT))

for directory in ("support", "tooling", "acceptance"):
    path = str(TEST_ROOT / directory)
    if path not in sys.path:
        sys.path.insert(0, path)
