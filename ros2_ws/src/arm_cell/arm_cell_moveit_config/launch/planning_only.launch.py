"""MoveIt robot-only planning; simulator and TF composition belong to bringup."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def compose(context):
    share = Path(get_package_share_directory("arm_cell_moveit_config"))
    description = Path(get_package_share_directory("arm_cell_description"))

    def config(name):
        return yaml.safe_load((share / "config" / name).read_text())

    planning_limits = config("joint_limits.yaml")
    planning_pipeline = config("ompl_planning.yaml")
    planning_profile = LaunchConfiguration("planning_profile").perform(context)
    if planning_profile:
        profile = yaml.safe_load(Path(planning_profile).read_text()) or {}
        acceleration = profile.get("motion", {}).get("planning_max_acceleration_rad_s2")
        if acceleration is not None:
            acceleration = float(acceleration)
            if acceleration <= 0.0:
                raise RuntimeError("planning acceleration seed must be positive")
            planning_limits.setdefault("joint_limits", {})
            for joint_name in (
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ):
                planning_limits["joint_limits"][joint_name] = {
                    "has_acceleration_limits": True,
                    "max_acceleration": acceleration,
                }
            planning_pipeline["request_adapters"] = (
                planning_pipeline["request_adapters"]
                + " default_planner_request_adapters/AddTimeOptimalParameterization"
            )

    parameters = [
        {
            "use_sim_time": True,
            "robot_description": xacro.process_file(
                str(description / "urdf/m0609_robotiq_2f85.urdf.xacro")
            ).toxml(),
            "robot_description_semantic": (
                share / "config/m0609_robotiq_2f85.srdf"
            ).read_text(),
            "robot_description_kinematics": config("kinematics.yaml"),
            "robot_description_planning": planning_limits,
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": planning_pipeline,
        },
        config("planning_only.yaml"),
    ]
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=parameters,
            remappings=[("joint_states", LaunchConfiguration("joint_states"))],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", str(share / "config/moveit.rviz")],
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("joint_states", default_value="/joint_states"),
            DeclareLaunchArgument("planning_profile", default_value=""),
            OpaqueFunction(function=compose),
        ]
    )
