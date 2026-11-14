"""Ordered Script Editor roles for the common Isaac backend.

Before Play owns the selected configuration snapshot. After Play reuses it;
Pre Build is deliberately independent and never runs implicitly at startup.
"""
import os
from pathlib import Path
from importlib import import_module
from .runtime_configuration import load_runtime_configuration

ROLES = {
    "pre_build": ("build_robot_usd.py",),
    "before_play": ("setup_scene.py", "snapshot_camera.py", "build_actiongraph.py"),
    "after_play": ("bootstrap_tool.py", "enable_simulation_ui_hub.py"),
    "after_play_headless": ("bootstrap_tool.py",),
}


def call_selected(reference, **services):
    module, name = reference.split(":")
    return getattr(import_module(module), name)(**services)


def run_step(filename, namespace):
    step = Path(__file__).parent / "runtime_steps" / filename
    namespace["__file__"] = str(step)
    exec(compile(step.read_text(encoding="utf-8"), str(step), "exec"), namespace)


def run_role(role, namespace):
    if role not in ROLES:
        raise ValueError(f"unsupported ARM Cell runtime role: {role}")
    if role in ("pre_build", "before_play") or "runtime_configuration" not in namespace:
        if role == "before_play":
            close = namespace.get("close_runtime")
            if callable(close):
                close()
        namespace["runtime_configuration"] = load_runtime_configuration()
    namespace["SFTWIN_RUNTIME_ROLE"] = role
    if role in ("after_play", "after_play_headless"):
        os.environ["SFTWIN_RUNTIME_MODE"] = "production"
    for filename in ROLES[role]:
        run_step(filename, namespace)
