"""Unit checks for the Isaac-hosted W05 Hub extension startup entrypoint."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType


SCRIPT = Path(__file__).parents[2] / "runtime_steps/enable_simulation_ui_hub.py"


def _load_script():
    spec = importlib.util.spec_from_file_location(
        name="enable_simulation_ui_hub", location=SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_hub_startup_registers_and_enables_the_repository_extension(
    tmp_path, monkeypatch
):
    relative_extension = Path("infra") / "isaac_sim" / "extensions"
    relative_extension /= "arm_cell.simulation_ui_hub"
    extension = tmp_path / relative_extension
    (extension / "config").mkdir(parents=True)
    (extension / "config/extension.toml").write_text("[package]\n")
    monkeypatch.setenv("SFTWIN_PROJECT_ROOT", str(tmp_path))

    calls = []
    nonlocal_enabled = [False]

    class Manager:
        def add_path(self, path):
            calls.append(("add_path", path))

        def is_extension_enabled(self, _extension_id):
            return nonlocal_enabled[0]

        def set_extension_enabled_immediate(self, extension_id, value):
            nonlocal_enabled[0] = value
            calls.append(("enable", extension_id, value))

    app = ModuleType("omni.kit.app")

    class KitApp:
        def get_extension_manager(self):
            return Manager()

    app.get_app = lambda: KitApp()
    kit = ModuleType("omni.kit")
    kit.__path__ = []
    kit.app = app
    omni = ModuleType("omni")
    omni.__path__ = []
    omni.kit = kit
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.kit", kit)
    monkeypatch.setitem(sys.modules, "omni.kit.app", app)

    result = _load_script().enable_simulation_ui_hub()
    repeated_result = _load_script().enable_simulation_ui_hub()

    assert result == str(extension)
    assert repeated_result == str(extension)
    assert calls == [
        ("add_path", str(extension.parent)),
        ("enable", "arm_cell.simulation_ui_hub", True),
        ("add_path", str(extension.parent)),
    ]
