"""Load the ARM Cell Simulation UI Hub into the active Isaac Kit process."""

import importlib
import os
from pathlib import Path
import sys


def enable_simulation_ui_hub():
    root_value = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not root_value:
        raise RuntimeError(
            "Run 1_before_play.py to configure the active project checkout"
        )
    hub_relative = "infra/isaac_sim/extensions/arm_cell.simulation_ui_hub"
    extension_path = Path(root_value).expanduser().resolve() / hub_relative
    if not (extension_path / "config/extension.toml").is_file():
        raise RuntimeError(
            "Simulation UI Hub extension is missing: " + str(extension_path)
        )

    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.add_path(str(extension_path.parent))
    extension_id = "arm_cell.simulation_ui_hub"
    # Play/Stop changes the simulation timeline, not the Kit application.
    # Keep the Hub alive for the whole Kit session so after_play can safely be
    # invoked again without asking Kit to tear down and recreate its IExt.
    if manager.is_extension_enabled(extension_id):
        return str(extension_path)

    # Kit's custom importer resolves the declared module name, but local
    # extensions with a ``python/`` source tree are not always added to
    # sys.path when registered dynamically from Script Editor.
    python_path = str(extension_path / "python")
    if python_path not in sys.path:
        sys.path.insert(0, python_path)
    package_path = Path(python_path) / "arm_cell_simulation_ui_hub"
    bytecode_cache = package_path / "__pycache__"
    if bytecode_cache.is_dir():
        for bytecode_path in bytecode_cache.glob("*.pyc"):
            bytecode_path.unlink(missing_ok=True)
    importlib.invalidate_caches()
    for module_name in (
        "arm_cell_simulation_ui_hub.sidecar",
        "arm_cell_simulation_ui_hub.runtime",
        "arm_cell_simulation_ui_hub.extension",
    ):
        module = sys.modules.get(module_name)
        if module is not None:
            importlib.reload(module)
    manager.set_extension_enabled_immediate(extension_id, True)
    return str(extension_path)


if __name__ == "__main__":
    enabled = enable_simulation_ui_hub()
    print("Simulation UI Hub is enabled and available in this Kit session.")
