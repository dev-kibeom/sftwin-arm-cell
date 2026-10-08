"""Role based Script Editor workflow for the live ARM Cell runtime."""

from pathlib import Path
import os

from shared.script_editor_bootstrap import module_root_from_project_root


ROLES = {
    "pre_build": ("build_robot_usd.py",),
    "before_play": ("setup_scene.py", "snapshot_camera.py", "build_actiongraph.py"),
    "after_play": ("bootstrap_gripper.py", "enable_simulation_ui_hub.py"),
}


def run_role(role, namespace):
    """Run a role's implementation steps in the persistent Script Editor globals."""
    if role not in ROLES:
        raise ValueError(f"unsupported ARM Cell runtime role: {role}")
    root_value = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not root_value:
        raise RuntimeError("Run 1_before_play.py before an ARM Cell runtime role")
    module_root = module_root_from_project_root(root_value)
    steps_root = module_root / "runtime_steps"
    namespace["SFTWIN_RUNTIME_ROLE"] = role
    if role == "after_play":
        os.environ["SFTWIN_RUNTIME_MODE"] = "production"
    for filename in ROLES[role]:
        step = steps_root / filename
        if not step.is_file():
            raise FileNotFoundError(f"ARM Cell runtime step is missing: {step}")
        print(f">>> ARM Cell runtime: {role} -> {filename}")
        namespace["__file__"] = str(step)
        source = compile(step.read_text(encoding="utf-8"), str(step), "exec")
        exec(source, namespace)
    print(f">>> ARM Cell runtime role complete: {role}")
