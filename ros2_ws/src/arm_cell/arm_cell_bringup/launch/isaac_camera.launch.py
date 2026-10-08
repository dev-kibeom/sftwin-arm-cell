"""Publish only the two static camera-overlay TF edges from a verified snapshot."""

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _transform_arguments(transform):
    translation = transform["translation_m"]
    quaternion = transform["quaternion_xyzw"]
    return [
        str(value)
        for value in (
            *translation,
            *quaternion,
            transform["parent_frame"],
            transform["child_frame"],
        )
    ]


def compose(context):
    snapshot_path = LaunchConfiguration("snapshot").perform(context).strip()
    if not snapshot_path:
        profile = Path(LaunchConfiguration("profile").perform(context))
        config = yaml.safe_load(profile.read_text()) or {}
        snapshot_path = config.get("camera_snapshot")
    if not snapshot_path:
        configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
        candidates = [Path(configured_root).expanduser()] if configured_root else []
        share = Path(get_package_share_directory("arm_cell_bringup"))
        candidates.extend(share.parents)
        for root in candidates:
            candidate = (
                root
                / ".local_artifacts/m0609_cell/camera_inspection/camera_snapshot.json"
            )
            if candidate.is_file():
                snapshot_path = str(candidate)
                break
    if not snapshot_path:
        raise RuntimeError(
            "camera overlay needs an inspector-generated camera_snapshot; no pose is assumed"
        )
    snapshot = json.loads(Path(snapshot_path).read_text())
    required = ("world_to_camera_link", "camera_link_to_optical")
    if any(key not in snapshot for key in required):
        raise RuntimeError("camera snapshot is incomplete")
    transforms = [snapshot[key] for key in required]
    if [(item["parent_frame"], item["child_frame"]) for item in transforms] != [
        ("world", "camera_link"),
        ("camera_link", "camera_color_optical_frame"),
    ]:
        raise RuntimeError("camera snapshot violates approved TF ownership")
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_camera_link",
            arguments=_transform_arguments(transforms[0]),
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_link_to_optical",
            arguments=_transform_arguments(transforms[1]),
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile", default_value=str(share / "config/isaac_camera.yaml")
            ),
            DeclareLaunchArgument("snapshot", default_value=""),
            OpaqueFunction(function=compose),
        ]
    )
