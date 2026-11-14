"""Fresh-load the workflow while retaining its Script Editor runtime objects."""
import importlib
from isaac_runtime import runtime_workflow


def run(role, namespace):
    importlib.invalidate_caches()
    importlib.reload(runtime_workflow).run_role(role, namespace)
