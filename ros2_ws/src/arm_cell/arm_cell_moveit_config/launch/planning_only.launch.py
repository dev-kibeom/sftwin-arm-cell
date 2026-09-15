"""MoveIt robot-only planning; simulator and TF composition belong to bringup."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_moveit_config"))
    description = Path(get_package_share_directory("arm_cell_description"))

    def config(name):
        return yaml.safe_load((share / "config" / name).read_text())

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
            "robot_description_planning": config("joint_limits.yaml"),
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": config("ompl_planning.yaml"),
        },
        config("planning_only.yaml"),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("joint_states", default_value="/joint_states"),
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
    )
