from pathlib import Path

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
        bootstrap.discover_module_root(CELL_ROOT / "inspect_camera.py", {}) == CELL_ROOT
    )
