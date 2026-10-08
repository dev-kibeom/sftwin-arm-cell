"""Fresh-load the small role dispatcher from the configured checkout."""

import importlib
import os
import sys

from shared.script_editor_bootstrap import module_root_from_project_root


def run(role, namespace):
    root_value = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not root_value:
        raise RuntimeError("Run 1_before_play.py before an ARM Cell runtime role")
    module_root = module_root_from_project_root(root_value)
    module_path = str(module_root)
    if module_path in sys.path:
        sys.path.remove(module_path)
    sys.path.insert(0, module_path)
    import shared.runtime_workflow as workflow

    importlib.invalidate_caches()
    workflow = importlib.reload(workflow)
    workflow.run_role(role, namespace)
