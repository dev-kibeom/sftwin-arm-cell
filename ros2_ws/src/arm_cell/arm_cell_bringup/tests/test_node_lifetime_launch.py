"""Standalone node lifetime regression checks."""

import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import pytest
import yaml


def _isolated_environment():
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(
        100 + (os.getpid() + int(time.monotonic() * 1000)) % 100
    )
    log_root = tempfile.mkdtemp(prefix="wu13-node-lifetime-")
    environment["ROS_HOME"] = log_root
    environment["ROS_LOG_DIR"] = os.path.join(log_root, "log")
    return environment


def _command_output(command, environment):
    return subprocess.run(
        command,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=3,
    ).stdout


def _assert_stays_visible(
    package, executable, node_name, endpoint_command, *, ros_arguments=()
):
    environment = _isolated_environment()
    process = subprocess.Popen(
        ["ros2", "run", package, executable, *ros_arguments],
        env=environment,
        start_new_session=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 10.0
        node_seen = False
        while time.monotonic() < deadline:
            if process.poll() is not None:
                stdout, stderr = process.communicate(timeout=1)
                pytest.fail(
                    f"{executable} exited with {process.returncode}: "
                    f"stdout={stdout!r} stderr={stderr!r}"
                )
            node_seen = node_seen or node_name in _command_output(
                ["ros2", "node", "list", "--no-daemon"], environment
            )
            if node_seen and endpoint_command(environment):
                return
            time.sleep(0.25)
        if node_seen:
            pytest.fail(f"endpoint for {node_name} did not become available")
        pytest.fail(f"{node_name} did not remain visible in the ROS graph")
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        process.communicate(timeout=1)


def test_vision_node_owns_node_while_spinning():
    def detect_target_service(environment):
        return "/vision/detect_target" in _command_output(
            ["ros2", "node", "info", "/vision_ros_ingress", "--no-daemon"],
            environment,
        )

    package_root = Path(__file__).parents[1]
    profile_path = package_root / "config/operational_profile.yaml"
    profile = yaml.safe_load(profile_path.read_text())
    with tempfile.TemporaryDirectory(prefix="wu13-vision-profile-") as temp_dir:
        params_path = Path(temp_dir) / "vision-parameters.yaml"
        params_path.write_text(
            yaml.safe_dump(
                {
                    "vision_ros_ingress": {
                        "ros__parameters": profile["vision"],
                    }
                }
            )
        )
        _assert_stays_visible(
            "arm_cell_vision",
            "detect_target_node",
            "/vision_ros_ingress",
            detect_target_service,
            ros_arguments=("--ros-args", "--params-file", str(params_path)),
        )


def test_orchestration_node_owns_node_while_spinning():
    def execute_cycle_action(environment):
        return "/orchestration/execute_cycle" in _command_output(
            ["ros2", "node", "info", "/orchestration_node", "--no-daemon"],
            environment,
        )

    _assert_stays_visible(
        "arm_cell_bt",
        "orchestration_node",
        "/orchestration_node",
        execute_cycle_action,
    )
