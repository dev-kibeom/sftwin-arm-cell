"""Selected entrypoints for the established M0609 Cell and Robotiq backend.

Only these selected implementations require the legacy sibling import path.
Common Startup and workflow never add a Cell directory to their search path.
"""
from pathlib import Path
import sys


def _run(filename, configuration, namespace):
    cell_root = Path(__file__).resolve().parent
    if str(cell_root) not in sys.path:
        sys.path.insert(0, str(cell_root))
    step = cell_root / "runtime_steps" / filename
    namespace["runtime_configuration"] = configuration
    namespace["__file__"] = str(step)
    exec(compile(step.read_text(encoding="utf-8"), str(step), "exec"), namespace)


def bootstrap_robotiq(*, configuration, namespace):
    _run("bootstrap_gripper.py", configuration, namespace)


def snapshot_camera(*, configuration, namespace):
    _run("snapshot_camera.py", configuration, namespace)


def build_action_graph(*, configuration, namespace):
    _run("build_actiongraph.py", configuration, namespace)
