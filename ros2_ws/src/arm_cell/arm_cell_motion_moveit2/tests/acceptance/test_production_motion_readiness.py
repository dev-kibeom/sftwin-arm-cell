"""Live production Motion readiness probe; requires the operational profile."""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import time

import rclpy
from arm_cell_interfaces.action import ExecuteTask
from rclpy.action import ActionClient

from managed_launch import ManagedLaunch


REQUIRED_NODES = {"/motion_node", "/isaac_motion_backend"}


def wait_for_readiness(node, action_client, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        observed_node_names = [
            f"{namespace.rstrip('/')}/{name}"
            for name, namespace in node.get_node_names_and_namespaces()
        ]
        node_names = set(observed_node_names)
        endpoints_ready = (
            node.count_publishers("/motion/state") >= 1
            and node.count_subscribers("/isaac/joint_states") >= 1
            and node.count_subscribers("/gripper/status") >= 1
            and action_client.server_is_ready()
        )
        if (
            REQUIRED_NODES.issubset(node_names)
            and observed_node_names.count("/motion_node") == 1
            and observed_node_names.count("/isaac_motion_backend") == 1
            and endpoints_ready
        ):
            return node_names
        rclpy.spin_once(node, timeout_sec=0.1)
    raise AssertionError(
        "production Motion readiness not reached: "
        f"nodes={sorted(observed_node_names)}, "
        f"motion_state_publishers={node.count_publishers('/motion/state')}, "
        f"joint_state_subscribers={node.count_subscribers('/isaac/joint_states')}, "
        f"gripper_status_subscribers={node.count_subscribers('/gripper/status')}, "
        f"execute_task_ready={action_client.server_is_ready()}"
    )


def probe_once(timeout):
    rclpy.init(args=[])
    node = rclpy.create_node(
        "production_motion_readiness_probe", use_global_arguments=False
    )
    action_client = ActionClient(node, ExecuteTask, "/motion/execute_task")
    try:
        node_names = wait_for_readiness(node, action_client, timeout)
        print("PRODUCTION_MOTION_READY")
        print(f"nodes={sorted(node_names)}")
        print("/motion/execute_task=ready")
        print("/motion/state=publisher-present")
        print("/isaac/joint_states=subscriber-present")
        print("/gripper/status=subscriber-present")
    finally:
        action_client.destroy()
        node.destroy_node()
        rclpy.shutdown()


def managed_probe(timeout, repeat, workspace):
    ros2 = shutil.which("ros2")
    if ros2 is None:
        raise RuntimeError("ros2 executable is not available")

    def ancestor_pids():
        ancestors = set()
        pid = os.getpid()
        while pid >= 1:
            ancestors.add(pid)
            try:
                stat = Path(f"/proc/{pid}/stat").read_text()
                command_end = stat.rfind(") ")
                fields = stat[command_end + 2:].split()
                pid = int(fields[1])
            except (FileNotFoundError, ValueError):
                break
        return ancestors

    def cardinality():
        argv_by_process = []
        for entry in Path("/proc").glob("[0-9]*"):
            if int(entry.name) in ancestor_pids():
                continue
            try:
                argv = (entry / "cmdline").read_bytes().split(b"\0")
                argv_by_process.append(
                    [argument.decode(errors="replace") for argument in argv if argument]
                )
            except (FileNotFoundError, PermissionError):
                continue
        return {
            "launch": sum(
                any(argument == "arm_cell_operational.launch.py" for argument in argv)
                for argv in argv_by_process
            ),
            "motion_node": sum(
                any(
                    argument.endswith(
                        "arm_cell_motion_moveit2/lib/arm_cell_motion_moveit2/motion_node"
                    )
                    for argument in argv
                )
                for argv in argv_by_process
            ),
        }

    def node_cardinality():
        result = subprocess.run(
            [ros2, "node", "list"], capture_output=True, text=True, check=False
        )
        output = result.stdout
        names = [line.strip() for line in output.splitlines() if line.startswith("/")]
        return {
            "motion_node": names.count("/motion_node"),
            "isaac_motion_backend": names.count("/isaac_motion_backend"),
        }

    baseline = {"process": cardinality(), "nodes": node_cardinality()}
    print(f"baseline_counts={baseline}")
    command = [ros2, "launch", "arm_cell_bringup", "arm_cell_operational.launch.py"]
    for iteration in range(repeat):
        with ManagedLaunch(command, cwd=workspace, env=os.environ.copy()):
            probe_once(timeout)
        # Process teardown is immediate; DDS graph removal can lag until the
        # participant lease expires, so require the baseline node cardinality
        # before declaring the repeated probe clean.
        deadline = time.monotonic() + 60.0
        remaining = {"process": cardinality(), "nodes": node_cardinality()}
        while remaining != baseline and time.monotonic() < deadline:
            time.sleep(0.25)
            remaining = {"process": cardinality(), "nodes": node_cardinality()}
        print(f"after_probe_{iteration + 1}_counts={remaining}")
        if remaining != baseline:
            raise AssertionError(
                f"launch process cardinality changed after iteration {iteration + 1}: "
                f"baseline={baseline}, remaining={remaining}"
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--manage-launch", action="store_true")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    arguments = parser.parse_args()
    if arguments.manage_launch:
        managed_probe(arguments.timeout, arguments.repeat, arguments.workspace)
    else:
        probe_once(arguments.timeout)
