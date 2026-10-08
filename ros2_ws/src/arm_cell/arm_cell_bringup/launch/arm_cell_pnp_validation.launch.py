"""Compose ROS for validation-only deterministic PnP.

The Isaac fixture runner and ROS registration helper are intentionally external
to ROS launch.  Execute the profile's ``runtime_runner_entrypoint`` in Isaac
Script Editor, then execute its ``registration_helper_entrypoint`` in a ROS
Humble terminal after this composition is live.  This launch file does not
claim to spawn simulator material or register it by itself.
"""

from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def profile_parameters(config, section):
    parameters = {
        "use_sim_time": config.get("use_sim_time", True),
        **config.get(section, {}),
    }
    return {key: value for key, value in parameters.items() if value != []}


def validate_approach_entry_tolerances(config):
    """Require finite, positive motion acceptance tolerances in the profile."""
    motion = config.get("motion")
    if not isinstance(motion, dict):
        raise RuntimeError("validation profile is missing the motion section")
    for key in (
        "approach_entry_position_tolerance_m",
        "place_approach_entry_position_tolerance_m",
        "approach_entry_orientation_tolerance_rad",
        "place_tracking_tolerance_rad",
    ):
        value = motion.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0.0
        ):
            raise RuntimeError(
                f"validation profile motion.{key} must be finite and positive"
            )


def compose(context):
    profile_path = Path(LaunchConfiguration("profile").perform(context))
    config = yaml.safe_load(profile_path.read_text()) or {}
    validate_approach_entry_tolerances(config)
    share = Path(get_package_share_directory("arm_cell_bringup"))
    home_config = (
        yaml.safe_load((share / "config/home_joint_positions.yaml").read_text()) or {}
    )
    home_joint_positions = home_config["home_joint_positions"]
    if len(home_joint_positions) != 6:
        raise RuntimeError("home_joint_positions must contain six arm joints")
    description = Path(get_package_share_directory("arm_cell_description"))
    moveit_share = Path(get_package_share_directory("arm_cell_moveit_config"))
    robot_description = xacro.process_file(
        str(description / "urdf/m0609_robotiq_2f85.urdf.xacro")
    ).toxml()
    robot_description_semantic = (
        moveit_share / "config/m0609_robotiq_2f85.srdf"
    ).read_text()
    kinematics = yaml.safe_load((moveit_share / "config/kinematics.yaml").read_text())
    kinematics_parameters = {
        f"robot_description_kinematics.arm.{key}": value
        for key, value in kinematics["arm"].items()
    }
    validation = config["validation"]
    target_id = config["orchestration"]["mission_target_id"]
    if (
        validation["registration_ttl_s"] <= 0
        or validation["registration_ttl_max_s"] > 60.0
    ):
        raise RuntimeError(
            "validation registration TTL is outside the governed contract"
        )

    nodes = [
        Node(
            package="arm_cell_vda",
            executable="external_state_mock_node",
            name="external_state_mock",
            output="screen",
            parameters=[profile_parameters(config, "vda")],
        ),
        Node(
            package="arm_cell_safety",
            executable="safety_node",
            name="safety_node",
            output="screen",
            parameters=[profile_parameters(config, "safety")],
        ),
        Node(
            package="arm_cell_vision",
            executable=validation["fixed_vision"]["executable"],
            name="fixed_detect_target_node",
            output="screen",
            parameters=[
                {
                    "use_sim_time": config.get("use_sim_time", True),
                    "target_id": target_id,
                }
            ],
        ),
        Node(
            package="arm_cell_motion_moveit2",
            executable="motion_node",
            name="motion_node",
            output="screen",
            parameters=[
                {
                    **profile_parameters(config, "motion"),
                    "home_joint_positions": home_joint_positions,
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                    **kinematics_parameters,
                }
            ],
        ),
        Node(
            package="arm_cell_bt",
            executable="orchestration_node",
            name="orchestration_node",
            output="screen",
            parameters=[
                {
                    **profile_parameters(config, "orchestration"),
                    "mission_target_id": target_id,
                    "recipe_directory": str(
                        share
                        / "config"
                        / config["orchestration"].get("recipe_directory", "recipes")
                    ),
                }
            ],
        ),
        Node(
            package="arm_cell_bringup",
            executable="pnp_planning_scene_runtime.py",
            name="pnp_planning_scene_runtime",
            output="screen",
            parameters=[
                {
                    "use_sim_time": config.get("use_sim_time", True),
                    "state_file": "/tmp/sftwin_pnp_validation/planning_scene_state.json",
                    "place_confirmation_file": "/tmp/sftwin_pnp_validation/place_confirmation.json",
                    "place_result_event_file": "/tmp/sftwin_pnp_validation/place_result_event.json",
                    "mission_completion_event_file": "/tmp/sftwin_pnp_validation/mission_completion_event.json",
                    "fixture_delete_request_file": "/tmp/sftwin_pnp_validation/fixture_delete_request.json",
                    "fixture_delete_ack_file": "/tmp/sftwin_pnp_validation/fixture_delete_ack.json",
                    "cleanup_delay_s": float(config["place"]["deletion_delay_s"]),
                    "attachment_link": config["validation"]["planning_scene"][
                        "attachment_link"
                    ],
                    "touch_links": config["validation"]["planning_scene"][
                        "touch_links"
                    ],
                }
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "launch/isaac_planning.launch.py")
            ),
            launch_arguments={
                "profile": str(share / "config/isaac_planning.yaml"),
                # Pass the already-resolved validation profile path through the
                # nested planning launch.  Keeping this as a nested
                # LaunchConfiguration loses the profile at runtime and leaves
                # MoveIt with ResolveConstraintFrames only.
                "planning_profile": str(profile_path),
                "static_scene": "true",
                "verify_nominal_state": "true",
                "scene_file": str(
                    Path(get_package_share_directory("arm_cell_moveit_config"))
                    / "config/m0609_pnp_static_scene.json"
                ),
            }.items(),
        ),
    ]
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value=str(share / "config/pnp_validation_profile.yaml"),
            ),
            OpaqueFunction(function=compose),
        ]
    )
