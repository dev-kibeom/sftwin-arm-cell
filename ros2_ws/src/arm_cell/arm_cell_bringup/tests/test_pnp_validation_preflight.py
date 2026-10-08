"""Static and mocked-live checks for the canonical validation preflight."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).parents[1]
SCRIPT = PACKAGE_ROOT / "scripts/pnp_validation_preflight.py"
REPO_ROOT = PACKAGE_ROOT.parents[3]


def _module():
    spec = importlib.util.spec_from_file_location("pnp_validation_preflight", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_static_preflight_accepts_repository_files_and_profile():
    module = _module()

    checks = module.validate_repository(REPO_ROOT, environ={})

    assert any("profile" in check for check in checks)
    assert any("executable" in check for check in checks)


def test_validation_runtime_script_is_executable_with_canonical_shebang():
    runtime_script = PACKAGE_ROOT / "scripts/pnp_planning_scene_runtime.py"

    assert runtime_script.stat().st_mode & 0o111
    assert runtime_script.read_text().splitlines()[0] == "#!/usr/bin/env python3"


def test_static_preflight_rejects_missing_validation_file(tmp_path):
    module = _module()
    with pytest.raises(module.PreflightError, match="canonical repo path"):
        module.validate_repository(tmp_path, environ={})


def test_workspace_preflight_reports_branch_head_and_dirty_state():
    module = _module()

    def runner(command, **kwargs):
        del kwargs
        output = {
            ("git", "branch", "--show-current"): "feature/wu14-task3-capture-adapter\n",
            ("git", "rev-parse", "HEAD"): "abc123\n",
            ("git", "status", "--porcelain"): " M user.py\n",
        }[tuple(command)]
        return SimpleNamespace(returncode=0, stdout=output, stderr="")

    with pytest.raises(module.PreflightError, match="dirty"):
        module.validate_workspace(
            REPO_ROOT,
            runner=runner,
            expected_branch="feature/wu14-task3-capture-adapter",
        )


def test_live_preflight_requires_fixed_node_and_services():
    module = _module()
    responses = {
        ("ros2", "node", "list"): "/fixed_detect_target_node\n",
        ("ros2", "service", "list"): (
            "/fixed_detect_target_node/register_target\n/vision/detect_target\n"
        ),
    }

    def runner(command, **kwargs):
        del kwargs
        return SimpleNamespace(
            returncode=0,
            stdout=responses[tuple(command)],
            stderr="",
        )

    checks = module.validate_live_ros(runner=runner)

    assert "fixed_detect_target_node" in " ".join(checks)


def test_live_preflight_rejects_missing_registration_service():
    module = _module()

    def runner(command, **kwargs):
        del kwargs
        if tuple(command) == ("ros2", "node", "list"):
            return SimpleNamespace(
                returncode=0, stdout="/fixed_detect_target_node\n", stderr=""
            )
        return SimpleNamespace(
            returncode=0, stdout="/vision/detect_target\n", stderr=""
        )

    with pytest.raises(module.PreflightError, match="register_target"):
        module.validate_live_ros(runner=runner)


@pytest.mark.parametrize(
    ("nodes", "message"),
    [
        ("", "zero"),
        (
            "/fixed_detect_target_node\n/fixed_detect_target_node\n",
            "duplicate",
        ),
        (
            "/fixed_detect_target_node\n/fixed_detect_target_node_legacy\n",
            "ambiguous",
        ),
    ],
)
def test_live_preflight_rejects_non_unique_fixed_vision_graph(nodes, message):
    module = _module()

    def runner(command, **kwargs):
        del kwargs
        if tuple(command) == ("ros2", "node", "list"):
            return SimpleNamespace(returncode=0, stdout=nodes, stderr="")
        return SimpleNamespace(
            returncode=0,
            stdout="/fixed_detect_target_node/register_target\n/vision/detect_target\n",
            stderr="",
        )

    with pytest.raises(module.PreflightError, match=message):
        module.validate_live_ros(runner=runner)


def test_live_preflight_rejects_duplicate_registration_service_listing():
    module = _module()

    def runner(command, **kwargs):
        del kwargs
        if tuple(command) == ("ros2", "node", "list"):
            return SimpleNamespace(
                returncode=0, stdout="/fixed_detect_target_node\n", stderr=""
            )
        return SimpleNamespace(
            returncode=0,
            stdout=(
                "/fixed_detect_target_node/register_target\n"
                "/fixed_detect_target_node/register_target\n"
                "/vision/detect_target\n"
            ),
            stderr="",
        )

    with pytest.raises(module.PreflightError, match="registration service"):
        module.validate_live_ros(runner=runner)
