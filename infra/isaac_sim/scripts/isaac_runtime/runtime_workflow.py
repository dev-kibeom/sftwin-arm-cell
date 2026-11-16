"""Ordered Script Editor roles for the common Isaac backend.

Before Play owns the selected configuration snapshot. After Play reuses it;
Pre Build remains independent. A deployment can explicitly opt in to preparing
missing generated model assets during host startup.
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


def prepare_startup_model(namespace, project_root):
    """Honor the selected Assembly's explicit first-run preparation policy."""
    from .model_build_configuration import ModelBuildConfiguration
    from .model_provenance import verify_current_artifact
    configuration = load_runtime_configuration()
    namespace['runtime_configuration'] = configuration
    if not configuration['model_configuration']['isaac'].get('build_missing_on_startup', False):
        return
    model = ModelBuildConfiguration.from_runtime(configuration, project_root)
    if all(path.is_file() for path in (model.generated_urdf, model.usd, model.provenance)):
        try:
            verify_current_artifact(model.source, model.generated_urdf, model.usd,
                                    model.provenance, xacro_arguments=model.xacro_arguments)
            return
        except ValueError:
            # The selected source changed: refresh with the same common producer.
            pass
    run_step('build_robot_usd.py', namespace)
