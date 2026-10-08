#!/usr/bin/env python3
"""Deterministic host-side preflight for live Isaac PnP validation."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import stat
import subprocess

import yaml


class PreflightError(RuntimeError):
    """Raised when live-validation prerequisites are not satisfied."""


def _run(command, *, cwd=None, runner=subprocess.run):
    result = runner(
        command,
        cwd=cwd,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise PreflightError(f"command failed: {' '.join(command)}: {detail}")
    return result.stdout.strip()


def _require(condition, message):
    if not condition:
        raise PreflightError(message)


def effective_ros_domain(environ=None):
    environment = os.environ if environ is None else environ
    value = environment.get("ROS_DOMAIN_ID", "0").strip()
    if not value.isdigit() or not 0 <= int(value) <= 232:
        raise PreflightError("effective ROS_DOMAIN_ID must be an integer in 0..232")
    return str(int(value))


def validate_repository(root, *, environ=None):
    root = Path(root).expanduser().resolve()
    environment = os.environ if environ is None else environ
    configured_root = environment.get("SFTWIN_PROJECT_ROOT")
    if configured_root:
        _require(
            Path(configured_root).expanduser().resolve() == root,
            "SFTWIN_PROJECT_ROOT does not match the canonical repository path",
        )
    _require(
        root.is_dir() and (root / ".git").exists(), "canonical repo path is invalid"
    )
    required_files = (
        "infra/isaac_sim/scripts/m0609_cell/shared/environment_setup.py",
        "infra/isaac_sim/scripts/m0609_cell/pnp_validation/entrypoints/start_validation.py",
        "ros2_ws/src/arm_cell/arm_cell_bringup/launch/arm_cell_pnp_validation.launch.py",
        "ros2_ws/src/arm_cell/arm_cell_bringup/scripts/register_pnp_fixture.py",
        "ros2_ws/src/arm_cell/arm_cell_bringup/scripts/pnp_validation_preflight.py",
        "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml",
    )
    for relative in required_files:
        _require((root / relative).is_file(), f"required file is missing: {relative}")

    scripts = (
        root / "ros2_ws/src/arm_cell/arm_cell_bringup/scripts/register_pnp_fixture.py",
        root
        / "ros2_ws/src/arm_cell/arm_cell_bringup/scripts/pnp_validation_preflight.py",
    )
    for script in scripts:
        mode = stat.S_IMODE(script.stat().st_mode)
        _require(mode & stat.S_IXUSR, f"script is not executable: {script}")
        _require(
            script.read_text().splitlines()[0] == "#!/usr/bin/env python3",
            f"script has no canonical shebang: {script}",
        )

    profile_path = Path(
        environment.get(
            "SFTWIN_PNP_PROFILE",
            root
            / "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml",
        )
    ).expanduser()
    if not profile_path.is_absolute():
        profile_path = root / profile_path
    _require(
        profile_path.is_file(), f"validation profile does not resolve: {profile_path}"
    )
    profile = yaml.safe_load(profile_path.read_text()) or {}
    for key in ("runtime_runner_entrypoint", "registration_helper_entrypoint"):
        relative = profile["validation"][key]
        _require(
            (root / relative).is_file(),
            f"profile path does not resolve: validation.{key}={relative}",
        )
    return [
        f"canonical repo: {root}",
        f"validation profile: {profile_path}",
        f"effective ROS_DOMAIN_ID: {effective_ros_domain(environment)}",
        "required validation scripts are present and executable",
    ]


def validate_workspace(
    root, *, runner=subprocess.run, expected_branch=None, expected_head=None
):
    branch = _run(("git", "branch", "--show-current"), cwd=root, runner=runner)
    head = _run(("git", "rev-parse", "HEAD"), cwd=root, runner=runner)
    status = _run(("git", "status", "--porcelain"), cwd=root, runner=runner)
    _require(not status, f"canonical workspace is dirty: {status!r}")
    if expected_branch:
        _require(branch == expected_branch, f"unexpected active branch: {branch}")
    if expected_head:
        _require(expected_head == head, f"unexpected HEAD: {head}")
    return [f"branch: {branch}", f"HEAD: {head}", "working tree: clean"]


def validate_ros_overlay(root, *, runner=subprocess.run):
    expected_prefix = (Path(root) / "ros2_ws/install/arm_cell_bringup").resolve()
    prefix = Path(
        _run(("ros2", "pkg", "prefix", "arm_cell_bringup"), runner=runner)
    ).resolve()
    _require(
        prefix == expected_prefix,
        f"ROS overlay prefix is not current workspace: {prefix}",
    )
    executables = _run(
        ("ros2", "pkg", "executables", "arm_cell_bringup"), runner=runner
    )
    _require(
        any("register_pnp_fixture.py" in line for line in executables.splitlines()),
        "ros2 pkg executables cannot discover register_pnp_fixture.py",
    )
    return [f"arm_cell_bringup prefix: {prefix}", "registration helper is discoverable"]


def validate_live_ros(*, runner=subprocess.run):
    nodes = _run(("ros2", "node", "list"), runner=runner)
    services = _run(("ros2", "service", "list"), runner=runner)
    node_names = [
        line.strip().rstrip("/") for line in nodes.splitlines() if line.strip()
    ]
    fixed_nodes = [
        name
        for name in node_names
        if name.rsplit("/", 1)[-1].startswith("fixed_detect_target_node")
    ]
    exact_fixed_nodes = [
        name for name in fixed_nodes if name == "/fixed_detect_target_node"
    ]
    if len(fixed_nodes) != len(exact_fixed_nodes):
        raise PreflightError(
            "ambiguous Fixed Vision validation runtime ownership; clean restart "
            f"required: {fixed_nodes}"
        )
    if not exact_fixed_nodes:
        raise PreflightError(
            "zero fixed_detect_target_node validation instances; clean restart "
            "and launch exactly one validation composition"
        )
    if len(exact_fixed_nodes) > 1:
        raise PreflightError(
            "duplicate fixed_detect_target_node validation instances; clean "
            "restart required before registration"
        )
    _require(
        node_names.count("/fixed_detect_target_node") == 1,
        "ambiguous Fixed Vision validation graph; clean restart required",
    )
    registration_services = [
        line.strip()
        for line in services.splitlines()
        if line.strip() == "/fixed_detect_target_node/register_target"
    ]
    if len(registration_services) > 1:
        raise PreflightError(
            "duplicate registration service authorities; clean restart required"
        )
    for service in (
        "/fixed_detect_target_node/register_target",
        "/vision/detect_target",
    ):
        _require(
            service in services.splitlines(),
            f"required ROS service is not discoverable: {service}",
        )
    return [
        "exactly one fixed_detect_target_node validation instance is discoverable",
        "registration and detect services are discoverable",
    ]


def run_preflight(
    root,
    *,
    live=False,
    runner=subprocess.run,
    expected_branch=None,
    expected_head=None,
):
    checks = validate_repository(root)
    checks += validate_workspace(
        root,
        runner=runner,
        expected_branch=expected_branch,
        expected_head=expected_head,
    )
    checks += validate_ros_overlay(root, runner=runner)
    if live:
        checks += validate_live_ros(runner=runner)
    return checks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--branch")
    parser.add_argument("--head")
    parser.add_argument("--live", action="store_true")
    args = parser.parse_args(argv)
    try:
        for check in run_preflight(
            args.repo_root,
            live=args.live,
            expected_branch=args.branch,
            expected_head=args.head,
        ):
            print(f"[PNP PREFLIGHT] PASS: {check}")
    except (OSError, KeyError, ValueError, PreflightError) as error:
        print(f"[PNP PREFLIGHT] FAIL: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
