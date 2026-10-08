"""Expose the Isaac extension package to deterministic host-side tests."""

from pathlib import Path
import sys

repository = Path(__file__).resolve().parents[6]
extension_python = (
    repository / "infra/isaac_sim/extensions/arm_cell.simulation_ui_hub/python"
)
if str(extension_python) not in sys.path:
    sys.path.insert(0, str(extension_python))
