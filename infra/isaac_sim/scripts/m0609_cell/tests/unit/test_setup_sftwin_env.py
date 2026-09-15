import importlib.util
from pathlib import Path

import pytest


MODULE = Path(__file__).parents[2] / "0_setup_sftwin_env.py"
spec = importlib.util.spec_from_file_location("setup_sftwin_env", MODULE)
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)

PROJECT_ROOT = Path(__file__).resolve().parents[6]
CELL_ROOT = PROJECT_ROOT / "infra/isaac_sim/scripts/m0609_cell"


def test_explicit_project_root_is_validated_without_discovery():
    root, source = setup.resolve_project_root(PROJECT_ROOT, environ={})
    assert root == PROJECT_ROOT
    assert source == "explicit project root"


def test_public_layout_root_does_not_require_private_agents_file(tmp_path):
    public_root = tmp_path / "public-arm-cell"
    (public_root / setup.CELL_RELATIVE_PATH / "shared").mkdir(parents=True)
    (public_root / "ros2_ws/src/arm_cell").mkdir(parents=True)

    assert setup.validate_project_root(public_root) == public_root.resolve()


def test_existing_environment_root_is_validated_when_no_explicit_root_is_supplied():
    root, source = setup.resolve_project_root(
        None, environ={"SFTWIN_PROJECT_ROOT": str(PROJECT_ROOT)}
    )
    assert root == PROJECT_ROOT
    assert source == "existing SFTWIN_PROJECT_ROOT"


def test_explicit_root_has_precedence_over_existing_environment_root():
    root, source = setup.resolve_project_root(
        PROJECT_ROOT, environ={"SFTWIN_PROJECT_ROOT": "/tmp"}
    )
    assert root == PROJECT_ROOT
    assert source == "explicit project root"


def test_invalid_explicit_or_environment_root_is_rejected():
    with pytest.raises(RuntimeError, match="not a validated"):
        setup.resolve_project_root("/tmp", environ={})
    with pytest.raises(RuntimeError, match="not a validated"):
        setup.resolve_project_root(None, environ={"SFTWIN_PROJECT_ROOT": "/tmp"})


def test_missing_explicit_and_environment_root_fails_without_heuristic_discovery():
    with pytest.raises(RuntimeError, match="No project root was supplied"):
        setup.resolve_project_root(None, environ={})


def test_configure_environment_sets_process_root_and_module_path(monkeypatch):
    environment = {}
    python_paths = []
    monkeypatch.setattr(setup.sys, "path", python_paths)
    result = setup.configure_environment(PROJECT_ROOT, environ=environment)
    assert result["SFTWIN_PROJECT_ROOT"] == str(PROJECT_ROOT)
    assert result["module_root"] == str(CELL_ROOT)
    assert environment["SFTWIN_PROJECT_ROOT"] == str(PROJECT_ROOT)
    assert python_paths == [str(CELL_ROOT)]
