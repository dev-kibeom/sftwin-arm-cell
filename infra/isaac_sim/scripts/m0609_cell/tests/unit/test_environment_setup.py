import importlib.util
from pathlib import Path

import pytest


MODULE = Path(__file__).parents[2] / "shared/environment_setup.py"
spec = importlib.util.spec_from_file_location("environment_setup", MODULE)
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


def test_live_environment_bootstrap_needs_no_diagnostic_profile(monkeypatch):
    environment = {}
    python_paths = []
    monkeypatch.setattr(setup.sys, "path", python_paths)

    result = setup.configure_live_environment(PROJECT_ROOT, environ=environment)

    assert result["SFTWIN_PROJECT_ROOT"] == str(PROJECT_ROOT)
    assert environment["ROS_DOMAIN_ID"] == "0"
    assert environment["SFTWIN_RUNTIME_MODE"] == "production"
    assert "SFTWIN_PNP_PROFILE" not in environment
    assert python_paths == [str(CELL_ROOT)]


def test_before_play_bootstraps_live_environment_without_running_setup_entrypoint(
    monkeypatch,
):
    import importlib
    import runpy
    import sys
    from types import ModuleType

    calls = []
    role_entrypoint = ModuleType("shared.role_entrypoint")
    role_entrypoint.run = lambda role, namespace: calls.append(role)
    monkeypatch.setitem(sys.modules, "shared.role_entrypoint", role_entrypoint)
    monkeypatch.setattr(importlib, "reload", lambda module: module)
    environment = monkeypatch.context()
    with environment as scoped:
        scoped.delenv("SFTWIN_PROJECT_ROOT", raising=False)
        scoped.delenv("ROS_DOMAIN_ID", raising=False)
        scoped.delenv("SFTWIN_PNP_PROFILE", raising=False)
        scoped.delenv("SFTWIN_RUNTIME_MODE", raising=False)
        runpy.run_path(str(CELL_ROOT / "1_before_play.py"))
        assert calls == ["before_play"]
        assert setup.os.environ["SFTWIN_PROJECT_ROOT"] == str(PROJECT_ROOT)
        assert setup.os.environ["ROS_DOMAIN_ID"] == "0"
        assert setup.os.environ["SFTWIN_RUNTIME_MODE"] == "production"
        assert "SFTWIN_PNP_PROFILE" not in setup.os.environ


def test_pre_build_uses_explicit_project_root_without_live_runtime_setup(monkeypatch):
    import importlib
    import runpy
    import sys
    from types import ModuleType

    calls = []
    role_entrypoint = ModuleType("shared.role_entrypoint")
    role_entrypoint.run = lambda role, namespace: calls.append(role)
    monkeypatch.setitem(sys.modules, "shared.role_entrypoint", role_entrypoint)
    monkeypatch.setattr(importlib, "reload", lambda module: module)
    with monkeypatch.context() as scoped:
        scoped.delenv("SFTWIN_PROJECT_ROOT", raising=False)
        scoped.delenv("ROS_DOMAIN_ID", raising=False)
        scoped.delenv("SFTWIN_PNP_PROFILE", raising=False)
        scoped.delenv("SFTWIN_RUNTIME_MODE", raising=False)
        runpy.run_path(str(CELL_ROOT / "0_pre_build.py"))
        assert calls == ["pre_build"]
        assert setup.os.environ["SFTWIN_PROJECT_ROOT"] == str(PROJECT_ROOT)
        assert "ROS_DOMAIN_ID" not in setup.os.environ
        assert "SFTWIN_PNP_PROFILE" not in setup.os.environ
        assert "SFTWIN_RUNTIME_MODE" not in setup.os.environ


def test_configure_environment_defaults_ros_domain_to_zero(monkeypatch):
    environment = {}
    monkeypatch.setattr(setup.sys, "path", [])

    result = setup.configure_environment(PROJECT_ROOT, environ=environment)

    assert result["ROS_DOMAIN_ID"] == "0"
    assert environment["ROS_DOMAIN_ID"] == "0"


def test_configure_environment_preserves_explicit_ros_domain():
    environment = {"ROS_DOMAIN_ID": "17"}

    result = setup.configure_environment(PROJECT_ROOT, environ=environment)

    assert result["ROS_DOMAIN_ID"] == "17"
    assert environment["ROS_DOMAIN_ID"] == "17"


def test_configure_environment_rejects_invalid_ros_domain():
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        setup.configure_environment(
            PROJECT_ROOT, environ={"ROS_DOMAIN_ID": "not-a-domain"}
        )


def test_configure_environment_sets_canonical_validation_profile(monkeypatch):
    environment = {}
    monkeypatch.setattr(setup.sys, "path", [])

    result = setup.configure_environment(PROJECT_ROOT, environ=environment)

    expected = (PROJECT_ROOT / setup.VALIDATION_PROFILE_RELATIVE_PATH).resolve()
    assert result["SFTWIN_PNP_PROFILE"] == str(expected)
    assert environment["SFTWIN_PNP_PROFILE"] == str(expected)


def test_configure_environment_accepts_valid_profile_override(tmp_path):
    override = tmp_path / "validation.yaml"
    override.write_text("validation: {}\n")
    environment = {"SFTWIN_PNP_PROFILE": str(override)}

    result = setup.configure_environment(PROJECT_ROOT, environ=environment)

    assert result["SFTWIN_PNP_PROFILE"] == str(override.resolve())
    assert environment["SFTWIN_PNP_PROFILE"] == str(override.resolve())


def test_configure_environment_rejects_invalid_profile_override():
    with pytest.raises(RuntimeError, match="SFTWIN_PNP_PROFILE"):
        setup.configure_environment(
            PROJECT_ROOT,
            environ={"SFTWIN_PNP_PROFILE": "/tmp/missing-pnp-profile.yaml"},
        )


def test_bootstrap_profile_is_consumable_by_task3_capture_config():
    import yaml

    environment = {}
    setup.configure_environment(PROJECT_ROOT, environ=environment)

    from pnp_validation.run_validation import capture_config_from_profile

    profile = yaml.safe_load(Path(environment["SFTWIN_PNP_PROFILE"]).read_text())
    capture_config = capture_config_from_profile(profile)

    assert capture_config is not None
    assert capture_config.expected_contact_width_mm == 70.0
