"""Compose ROS processes for the externally started Isaac simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def compose(context):
    config = yaml.safe_load(
        Path(LaunchConfiguration("profile").perform(context)).read_text()
    )
    description = Path(get_package_share_directory("arm_cell_description"))
    model = {
        "robot_description": xacro.process_file(
            str(description / "urdf/m0609_robotiq_2f85.urdf.xacro")
        ).toxml(),
        "use_sim_time": True,
    }
    pose = config["world_to_base"]
    arguments = ["--frame-id", "world", "--child-frame-id", "base_link"]
    for key in ("x", "y", "z", "roll", "pitch", "yaw"):
        arguments.extend(["--" + key, str(pose[key])])
    moveit = Path(get_package_share_directory("arm_cell_moveit_config"))
    static_loader = Node(
        package="arm_cell_moveit_config",
        executable="load_static_scene.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "scene_file": str(
                    Path(get_package_share_directory("arm_cell_moveit_config"))
                    / "config/m0609_static_scene.json"
                ),
            }
        ],
    )
    actions = [
        Node(
            package="arm_cell_sim_adapter",
            executable="joint_state_adapter",
            output="screen",
            parameters=[model],
            remappings=[
                ("raw_joint_states", config["raw_joint_states"]),
                ("joint_states", config["joint_states"]),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[model],
            remappings=[("joint_states", config["joint_states"])],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base",
            arguments=arguments,
            parameters=[{"use_sim_time": True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit / "launch/planning_only.launch.py")
            ),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz"),
                "joint_states": config["joint_states"],
            }.items(),
        ),
    ]
    if (
        LaunchConfiguration("static_scene", default="false").perform(context).lower()
        == "true"
    ):
        actions.extend(
            [
                static_loader,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=static_loader,
                        on_exit=[
                            EmitEvent(
                                event=Shutdown(reason="static scene loader exited")
                            )
                        ],
                    )
                ),
            ]
        )
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("static_scene", default_value="false"),
            DeclareLaunchArgument(
                "profile", default_value=str(share / "config/isaac_planning.yaml")
            ),
            OpaqueFunction(function=compose),
        ]
    )
