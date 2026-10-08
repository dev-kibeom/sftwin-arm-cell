from pathlib import Path
import sys
from types import ModuleType

import pytest

from shared import script_editor_bootstrap as bootstrap


PROJECT_ROOT = Path(__file__).resolve().parents[6]
CELL_ROOT = PROJECT_ROOT / "infra/isaac_sim/scripts/m0609_cell"


def test_explicit_project_root_resolves_cell_module_root():
    assert bootstrap.module_root_from_project_root(PROJECT_ROOT) == CELL_ROOT


def test_explicit_invalid_project_root_is_rejected():
    with pytest.raises(RuntimeError, match="not a directory"):
        bootstrap.module_root_from_project_root("/missing/sftwin")


def test_script_editor_temp_path_requires_explicit_project_root():
    with pytest.raises(RuntimeError, match="SFTWIN_PROJECT_ROOT"):
        bootstrap.discover_module_root("/tmp/carb.o6orAJ/script_1789079845.py", {})


def test_script_editor_temp_path_uses_explicit_project_root():
    assert (
        bootstrap.discover_module_root(
            "/tmp/carb.o6orAJ/script_1789079845.py",
            {"SFTWIN_PROJECT_ROOT": str(PROJECT_ROOT)},
        )
        == CELL_ROOT
    )


def test_source_path_resolution_does_not_depend_on_current_directory():
    assert (
        bootstrap.discover_module_root(
            CELL_ROOT / "camera_tooling/entrypoints/inspect_camera.py", {}
        )
        == CELL_ROOT
    )


def test_package_import_cache_can_be_cleared_without_removing_prefix_siblings(
    monkeypatch,
):
    package = "_test_graph_builder"
    names = (package, f"{package}.contract", f"{package}_other")
    for name in names:
        monkeypatch.setitem(sys.modules, name, ModuleType(name))

    bootstrap.clear_imported_package(package)

    assert package not in sys.modules
    assert f"{package}.contract" not in sys.modules
    assert f"{package}_other" in sys.modules


def test_package_bytecode_cache_is_removed_for_script_editor_reload(tmp_path):
    package = tmp_path / "gripper_runtime"
    cache = package / "__pycache__"
    cache.mkdir(parents=True)
    bytecode = cache / "session.cpython-310.pyc"
    bytecode.write_bytes(b"stale bytecode")

    bootstrap.clear_package_bytecode(package)

    assert not bytecode.exists()
