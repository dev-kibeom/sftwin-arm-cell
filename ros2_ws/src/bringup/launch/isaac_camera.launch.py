"""Publish only the two static camera-overlay TF edges from a verified snapshot."""

import importlib.util
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import yaml
from arm_cell_bringup.runtime import load_profile


def _load_console_output_format():
    helper_path = (
        Path(get_package_share_directory("arm_cell_motion"))
        / "launch/console_output_format.py"
    )
    spec = importlib.util.spec_from_file_location(
        "arm_cell_console_output_format", helper_path
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load console formatter from {helper_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.ConsoleOutputFormat("{line}")


CONSOLE_OUTPUT_FORMAT = _load_console_output_format()


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
    runtime_path = context.launch_configurations.get('runtime_profile', '') or os.environ.get('ARM_CELL_RUNTIME_CONFIG', '')
    camera = load_profile(runtime_path)['cell']['camera'] if runtime_path else None
    snapshot_path = camera['snapshot'] if camera is not None else LaunchConfiguration("snapshot").perform(context).strip()
    if not snapshot_path:
        profile = Path(LaunchConfiguration("profile").perform(context))
        config = yaml.safe_load(profile.read_text()) or {}
        snapshot_path = config.get("camera_snapshot")
    if not snapshot_path and camera is None:
        candidate = (
            Path(get_package_share_directory("arm_cell_sim_adapter"))
            / "config/isaac_camera_stage_snapshot.json"
        )
        if candidate.is_file():
            snapshot_path = str(candidate)
    if not snapshot_path:
        raise RuntimeError(
            "camera overlay needs an inspector-generated camera_snapshot; no pose is assumed"
        )
    snapshot = json.loads(Path(snapshot_path).read_text())
    if camera is not None:
        if snapshot.get('camera_prim') != camera['prim_path']:
            raise RuntimeError('camera snapshot does not match selected Cell camera prim')
        if snapshot.get('camera_binding') != {key: value for key, value in camera.items() if key != 'snapshot'}:
            raise RuntimeError('camera snapshot does not match selected Cell camera configuration')
    required = ("world_to_camera_link", "camera_link_to_optical")
    if any(key not in snapshot for key in required):
        raise RuntimeError("camera snapshot is incomplete")
    transforms = [snapshot[key] for key in required]
    if [(item["parent_frame"], item["child_frame"]) for item in transforms] != [
        ("world", camera['link_frame_id'] if camera else "camera_link"),
        (camera['link_frame_id'] if camera else "camera_link", camera['frame_id'] if camera else "camera_color_optical_frame"),
    ]:
        raise RuntimeError("camera snapshot violates approved TF ownership")
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_camera_link",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            arguments=_transform_arguments(transforms[0]),
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_link_to_optical",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            arguments=_transform_arguments(transforms[1]),
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "RCUTILS_CONSOLE_OUTPUT_FORMAT",
                "[{severity}] [{name}] {message}",
            ),
            SetEnvironmentVariable("TZ", "UTC"),
            DeclareLaunchArgument(
                "profile", default_value=str(share / "config/isaac_camera.yaml")
            ),
            DeclareLaunchArgument("snapshot", default_value=""),
            DeclareLaunchArgument("runtime_profile", default_value=EnvironmentVariable("ARM_CELL_RUNTIME_CONFIG", default_value="")),
            OpaqueFunction(function=compose),
        ]
    )
