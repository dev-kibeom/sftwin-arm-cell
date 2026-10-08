#!/usr/bin/env python3
"""Register an Isaac-produced deterministic PnP fixture handoff from ROS Humble.

This is intentionally the only registration client.  It must run in the ROS
Python environment, not inside Isaac Sim's Python 3.11 process.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import tempfile

EXPECTED_SCHEMA = "wu14-pnp-registration/v1"


def load_handoff(path: str | Path) -> dict:
    data = json.loads(Path(path).read_text())
    if data.get("schema_version") != EXPECTED_SCHEMA:
        raise ValueError("unsupported WU-14 registration handoff schema")
    if data.get("status") != "FIXTURE_READY_FOR_REGISTRATION":
        raise ValueError("handoff is not ready for registration")
    request = data.get("request", {})
    pose = request.get("pose", {})
    required = ("run_id", "target_id", "ttl_s")
    if (
        any(not request.get(key) for key in required)
        or pose.get("frame_id") != "base_link"
    ):
        raise ValueError("handoff request context is incomplete")
    if float(request["ttl_s"]) <= 0.0 or float(request["ttl_s"]) > 60.0:
        raise ValueError("handoff TTL must be in (0, 60]")
    return data


def request_from_handoff(data: dict, service_type=None):
    """Map the file seam to the existing Task 1/3 ROS service contract."""
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PoseStamped

    payload = data["request"]
    pose_data = payload["pose"]
    request = service_type() if service_type is not None else _request_type()()
    request.target_id = payload["target_id"]
    request.run_id = payload["run_id"]
    request.pose = PoseStamped()
    request.pose.header.frame_id = pose_data["frame_id"]
    request.pose.pose.position.x = float(pose_data["x"])
    request.pose.pose.position.y = float(pose_data["y"])
    request.pose.pose.position.z = float(pose_data["z"])
    request.pose.pose.orientation.z = math.sin(float(pose_data["yaw"]) / 2.0)
    request.pose.pose.orientation.w = math.cos(float(pose_data["yaw"]) / 2.0)
    request.has_target_yaw = bool(payload["has_target_yaw"])
    ttl_s = float(payload["ttl_s"])
    request.ttl = Duration(sec=int(ttl_s), nanosec=int(round((ttl_s % 1.0) * 1e9)))
    return request


def _request_type():
    from arm_cell_vision.srv import RegisterFixedTarget

    return RegisterFixedTarget.Request


def validate_receipt(response):
    if (
        not response.accepted
        or response.receipt_sequence <= 0
        or response.receipt_time_ns <= 0
    ):
        raise RuntimeError(
            "Fixed Vision registration receipt rejected: " f"{response.diagnostic}"
        )
    return response


def apply_fixture_to_planning_scene(node, handoff, service_name):
    """Add the accepted fixture as a validation-only world collision object."""
    from moveit_msgs.srv import ApplyPlanningScene

    from pnp_planning_scene import make_fixture_world_diff

    client = node.create_client(ApplyPlanningScene, service_name)
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("MoveIt planning-scene service is unavailable")
    future = client.call_async(make_fixture_world_diff(handoff))
    rclpy = __import__("rclpy")
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None or not future.result().success:
        raise RuntimeError("MoveIt rejected the validation fixture collision object")


def write_planning_scene_state(handoff, path):
    """Publish the accepted handoff to the validation-only runtime bridge."""
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(handoff, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return destination


def register(
    path: str | Path,
    service_name="/fixed_detect_target_node/register_target",
    planning_scene_service="/apply_planning_scene",
    planning_scene_state="/tmp/sftwin_pnp_validation/planning_scene_state.json",
):
    import rclpy
    from arm_cell_vision.srv import RegisterFixedTarget

    data = load_handoff(path)
    rclpy.init()
    node = rclpy.create_node("pnp_fixture_registration_helper")
    try:
        client = node.create_client(RegisterFixedTarget, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Fixed Vision registration service is unavailable")
        request = request_from_handoff(data, RegisterFixedTarget.Request)
        request.pose.header.stamp = node.get_clock().now().to_msg()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("Fixed Vision registration call did not complete")
        response = future.result()
        validate_receipt(response)
        apply_fixture_to_planning_scene(node, data, planning_scene_service)
        write_planning_scene_state(data, planning_scene_state)
        print(
            "[PnP Diagnostic] REGISTRATION_ACCEPTED "
            f"run_id={data['run_id']} target_id={data['target_id']} "
            f"receipt_sequence={response.receipt_sequence} "
            f"receipt_time_ns={response.receipt_time_ns}"
        )
        return response
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    parser.add_argument(
        "--service",
        default="/fixed_detect_target_node/register_target",
    )
    parser.add_argument(
        "--planning-scene-service",
        default="/apply_planning_scene",
    )
    parser.add_argument(
        "--planning-scene-state",
        default=os.environ.get(
            "SFTWIN_PNP_PLANNING_SCENE_STATE",
            "/tmp/sftwin_pnp_validation/planning_scene_state.json",
        ),
    )
    args = parser.parse_args(argv)
    try:
        register(
            args.manifest,
            args.service,
            args.planning_scene_service,
            args.planning_scene_state,
        )
    except (OSError, ValueError, RuntimeError) as error:
        print(f"[PnP Diagnostic] REGISTRATION_FAILED: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
