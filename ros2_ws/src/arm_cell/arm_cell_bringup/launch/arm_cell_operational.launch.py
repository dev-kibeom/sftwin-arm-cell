"""Compose the supported software-only ARM Cell operational profile."""

import fcntl
import importlib.util
import json
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _load_console_output_format():
    helper_path = (
        Path(get_package_share_directory("arm_cell_moveit_config"))
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
PLANNING_SCENE_LOGGER = "moveit_planning_scene.planning_scene"


def profile_parameters(config, section):
    """Return the exact profile values passed to a composed runtime node."""
    parameters = {
        "use_sim_time": config.get("use_sim_time", True),
        **config.get(section, {}),
    }
    if section == "vision":
        parameters["material_definitions"] = config.get("material_definitions", {})
    # launch_ros cannot encode an empty YAML sequence from a parameter dict;
    # omitting it preserves the node's declared empty-vector default.
    return {key: value for key, value in parameters.items() if value != []}


def vision_profile_parameters(config, profile_path):
    """Resolve the selected Calibration path relative to the bringup package root."""
    calibration_file = config.get("vision", {}).get("calibration_file")
    if not isinstance(calibration_file, str) or not calibration_file.strip():
        raise RuntimeError("operational profile requires vision.calibration_file")
    parameters = profile_parameters(config, "vision")
    path = Path(calibration_file)
    if not path.is_absolute():
        path = Path(profile_path).resolve().parent.parent / path
    parameters["calibration_file"] = str(path.resolve())
    return parameters


PROJECT_DEBUG_LOGGERS = {
    "arm_cell_gateway": ("material_handoff_node",),
    "arm_cell_safety": ("safety_node",),
    "arm_cell_vision": ("detect_target_node",),
    "arm_cell_motion": (
        "motion_node",
        "isaac_motion_backend",
        "motion_core",
        "motion_action_server",
        "task_executor",
    ),
    "arm_cell_orchestration": ("orchestration_node",),
}
SUPPORTED_DIAGNOSTIC_COMPONENTS = frozenset(
    {"motion", "vision", "safety", "orchestration"}
)
DIAGNOSTICS_DEFAULT = "__log_level_default__"
MOVEIT_RAW_LOG_LEVELS = (
    "ompl:=fatal",
    "moveit_kinematic_constraints.kinematic_constraints:=warn",
    "moveit_collision_detection_fcl.collision_common:=warn",
)
# Keep the spdlog process sink at DEBUG without enabling framework chatter.
DEBUG_FRAMEWORK_LOGGERS = (
    "rcl",
    "rclcpp",
    "rcl_action",
    "rclcpp_action",
    "rmw",
    "rmw_fastrtps_cpp",
    "move_group",
    "moveit",
    "moveit_ros",
    "moveit_core",
    "pluginlib",
    "kdl_parser",
    "kinematics_plugin_loader",
    "robot_state_publisher",
    "moveit_robot_model",
    "moveit_kinematics_base",
    "moveit_constraint_samplers",
)


def parse_diagnostics(value):
    """Resolve the operator's component diagnostic selection."""
    selection = value.strip()
    if selection == "none":
        return frozenset()
    if selection == "all":
        return SUPPORTED_DIAGNOSTIC_COMPONENTS
    components = [component.strip() for component in selection.split(",")]
    if any(not component for component in components):
        raise RuntimeError(
            "diagnostics must be 'none', 'all', or a comma-separated list of "
            "motion,vision,safety,orchestration"
        )
    if "all" in components or "none" in components:
        raise RuntimeError("diagnostics values 'all' and 'none' must be used alone")
    if len(set(components)) != len(components):
        raise RuntimeError("diagnostics contains a duplicate component name")
    unknown = sorted(set(components) - SUPPORTED_DIAGNOSTIC_COMPONENTS)
    if unknown:
        raise RuntimeError(
            "diagnostics contains unsupported component(s): " + ",".join(unknown)
        )
    return frozenset(components)


def resolve_diagnostics(value, log_level):
    """Use log level only when the operator omitted diagnostics."""
    if value == DIAGNOSTICS_DEFAULT:
        value = "all" if log_level == "debug" else "none"
    return parse_diagnostics(value)


def project_artifacts_directory():
    """Locate the checkout root so diagnostics do not depend on process cwd."""
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    candidates = [Path(configured_root).expanduser()] if configured_root else []
    candidates.extend((Path.cwd(), Path(__file__).resolve()))
    for candidate in candidates:
        candidate = candidate.resolve()
        for root in (candidate, *candidate.parents):
            if (root / "ros2_ws/src/arm_cell").is_dir() and (
                root / "infra/isaac_sim/scripts/m0609_cell"
            ).is_dir():
                return str(root / ".local_artifacts")
            if (root / "src/arm_cell").is_dir() and (
                root.parent / "infra/isaac_sim/scripts/m0609_cell"
            ).is_dir():
                return str(root.parent / ".local_artifacts")
    return str(Path.cwd() / ".local_artifacts")


def _git_provenance(artifact_directory):
    """Capture source checkout revision, not the running binary revision."""
    result = {
        "source": "git worktree",
        "source_root": None,
        "revision": None,
        "dirty_state": "unavailable",
        "unavailable_reason": None,
    }
    project_root = Path(
        os.environ.get("SFTWIN_PROJECT_ROOT", Path(artifact_directory).resolve().parent)
    )
    result["source_root"] = str(project_root.resolve())
    try:
        revision = subprocess.run(
            ["git", "-C", str(project_root), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            timeout=0.1,
            check=False,
        )
        if revision.returncode != 0:
            result["unavailable_reason"] = "revision_command_failed"
            return result
        result["revision"] = revision.stdout.strip()
        status = subprocess.run(
            ["git", "-C", str(project_root), "status", "--porcelain"],
            capture_output=True,
            text=True,
            timeout=0.1,
            check=False,
        )
        if status.returncode != 0:
            result["revision"] = None
            result["unavailable_reason"] = "status_command_failed"
            return result
        result["dirty_state"] = "dirty" if status.stdout.strip() else "clean"
    except FileNotFoundError:
        result["unavailable_reason"] = "git_unavailable"
    except subprocess.TimeoutExpired:
        result["revision"] = None
        result["unavailable_reason"] = "git_query_timeout"
    except OSError:
        result["revision"] = None
        result["unavailable_reason"] = "git_query_failed"
    return result


def join_live_test_session(artifact_directory):
    """Join only one explicit open Isaac descriptor; never guess by recency."""
    root = Path(artifact_directory) / "diagnostic_records"
    descriptors = []
    unreadable = False
    if root.is_dir():
        for descriptor in root.glob("[0-9]*/**/session.json"):
            try:
                data = json.loads(descriptor.read_text(encoding="utf-8"))
                if not isinstance(data, dict):
                    unreadable = True
                    continue
            except (OSError, ValueError):
                unreadable = True
                continue
            if data.get("status") != "open" or not data.get("session_id"):
                continue
            participants = data.get("participants", [])
            if not isinstance(participants, list):
                unreadable = True
                continue
            isaac = next(
                (
                    item
                    for item in participants
                    if isinstance(item, dict) and item.get("process") == "isaac"
                ),
                None,
            )
            try:
                pid = int(str(isaac.get("instance_id", "")).split("-", 1)[1])
                os.kill(pid, 0)
                process_stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
                current_start_token = process_stat.rsplit(")", 1)[1].split()[19]
                if current_start_token != str(isaac.get("process_start_token", "")):
                    continue
            except (AttributeError, IndexError, ValueError, OSError):
                continue
            descriptors.append((descriptor, data))
    if len(descriptors) != 1 or unreadable:
        return None
    descriptor, data = descriptors[0]
    try:
        own_stat = Path(f"/proc/{os.getpid()}/stat").read_text(encoding="utf-8")
        own_start_token = own_stat.rsplit(")", 1)[1].split()[19]
    except (OSError, IndexError):
        return None
    instance_id = f"ros-bringup-{os.getpid()}-{own_start_token}"
    participant = {
        "process": "ros_bringup",
        "instance_id": instance_id,
        "process_start_token": own_start_token,
        "status": "participating",
        "git_provenance": _git_provenance(artifact_directory),
    }
    recording_locations = {
        "snapshots": str(
            Path(artifact_directory)
            / "snapshots"
            / descriptor.parent.parent.name
            / descriptor.parent.name
        ),
        "vision_diagnostic_records": str(descriptor.parent / "vision"),
        "motion_planning_details": str(
            descriptor.parent
            / "motion"
            / "motion_planning_records"
            / "motion_planning_details"
        ),
        "motion_planning_summaries": str(
            descriptor.parent
            / "motion"
            / "motion_planning_records"
            / "motion_planning_summaries"
        ),
        "pick_execution_records": os.environ.get(
            "ARM_CELL_PICK_EXECUTION_RECORD_DIR",
            str(descriptor.parent / "motion" / "pick_execution_records"),
        ),
    }
    temporary = descriptor.with_suffix(".json.tmp")
    try:
        with Path(str(descriptor) + ".lock").open("a+") as lock_stream:
            fcntl.flock(lock_stream, fcntl.LOCK_EX)
            latest = json.loads(descriptor.read_text(encoding="utf-8"))
            if (
                latest.get("status") != "open"
                or latest.get("session_id") != data["session_id"]
            ):
                return None
            participants = latest.setdefault("participants", [])
            if not any(item.get("instance_id") == instance_id for item in participants):
                participants.append(participant)
            # These are writer destinations, not a claim that files already
            # exist or that shared snapshot directories belong to this session.
            latest.setdefault("recording_locations", {}).update(recording_locations)
            temporary.write_text(
                json.dumps(latest, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, descriptor)
            fcntl.flock(lock_stream, fcntl.LOCK_UN)
    except (AttributeError, OSError, TypeError, ValueError):
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        return None
    return descriptor.parent


def vision_snapshot_run_directory(artifact_directory, live_session_root):
    """Reuse the joined session's date/time path for Vision Snapshots."""
    if live_session_root is None:
        return None
    session_root = Path(live_session_root)
    return (
        Path(artifact_directory)
        / "snapshots"
        / session_root.parent.name
        / session_root.name
    )


def node_ros_arguments(
    package,
    log_level,
    action_logger_level="warn",
    planning_scene_log_level="info",
):
    """Apply the selected default level to console and file logging sinks."""
    arguments = ["--log-level", log_level]
    if log_level == "debug":
        for logger_name in DEBUG_FRAMEWORK_LOGGERS:
            arguments.extend(["--log-level", f"{logger_name}:=warn"])
    if package == "arm_cell_motion":
        for logger_setting in MOVEIT_RAW_LOG_LEVELS:
            arguments.extend(["--log-level", logger_setting])
        arguments.extend(["--log-level", "move_group_interface:=warn"])
        arguments.extend(
            [
                "--log-level",
                f"{PLANNING_SCENE_LOGGER}:={planning_scene_log_level}",
            ]
        )
    if log_level == "debug":
        for logger_name in PROJECT_DEBUG_LOGGERS.get(package, ()):
            arguments.extend(["--log-level", f"{logger_name}:=debug"])
    action_logger = {
        "arm_cell_motion": (
            "isaac_motion_backend.rclcpp_action",
            "motion_node.rclcpp_action",
        ),
        "arm_cell_orchestration": ("orchestration_node.rclcpp_action",),
    }.get(package, ())
    for logger_name in action_logger:
        arguments.extend(["--log-level", f"{logger_name}:={action_logger_level}"])
    return arguments


def compose(context):
    selected_log_level = LaunchConfiguration("log_level").perform(context)
    selected_planning_scene_log_level = LaunchConfiguration(
        "planning_scene_log_level"
    ).perform(context)
    selected_diagnostics = resolve_diagnostics(
        LaunchConfiguration("diagnostics", default=DIAGNOSTICS_DEFAULT).perform(
            context
        ),
        selected_log_level,
    )
    artifact_directory = Path(project_artifacts_directory())
    live_session_root = join_live_test_session(artifact_directory)
    live_session_id = None
    if live_session_root:
        try:
            live_session_id = json.loads(
                (live_session_root / "session.json").read_text(encoding="utf-8")
            ).get("session_id")
        except (OSError, ValueError, AttributeError):
            live_session_root = None
        if not live_session_id:
            live_session_root = None
    vision_artifact_directory = (
        live_session_root / "vision"
        if live_session_root
        else artifact_directory / "observability" / "vision"
    )
    snapshot_run_directory = vision_snapshot_run_directory(
        artifact_directory, live_session_root
    )
    motion_artifact_directory = (
        live_session_root / "motion"
        if live_session_root
        else artifact_directory / "observability"
    )
    safety_artifact_directory = (
        live_session_root / "safety"
        if live_session_root
        else artifact_directory / "observability" / "safety"
    )
    orchestration_artifact_directory = (
        live_session_root / "orchestration"
        if live_session_root
        else artifact_directory / "observability" / "orchestration"
    )
    selected_action_logger_level = LaunchConfiguration("action_logger_level").perform(
        context
    )
    profile_path = Path(LaunchConfiguration("profile").perform(context))
    config = yaml.safe_load(profile_path.read_text()) or {}
    material_handoff_config = dict(config.get("material_handoff", {}))
    motion_parameters = profile_parameters(config, "motion")
    trajectory_file = Path(motion_parameters["place_waypoint_trajectory_file"])
    if not trajectory_file.is_absolute():
        trajectory_file = Path(artifact_directory).parent / trajectory_file
    motion_parameters["place_waypoint_trajectory_file"] = str(trajectory_file.resolve())
    share = Path(get_package_share_directory("arm_cell_bringup"))
    home_config = (
        yaml.safe_load((share / "config/home_joint_positions.yaml").read_text()) or {}
    )
    home_joint_positions = home_config["home_joint_positions"]
    if len(home_joint_positions) != 6:
        raise RuntimeError("home_joint_positions must contain six arm joints")
    use_sim_time = config.get("use_sim_time", True)
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
    readiness = config.get("readiness", {})
    mission_target_id = config["orchestration"]["mission_target_id"]
    detector_profile = config["vision"]["detector_profiles"][mission_target_id]
    observation_reference = detector_profile.get("observation_reference")
    if not isinstance(observation_reference, str) or not observation_reference:
        raise RuntimeError(
            "mission detector profile must declare its observation_reference"
        )
    include_simulation = (
        LaunchConfiguration("include_simulation").perform(context).lower() == "true"
    )
    if not readiness.get("fail_closed_until_inputs_fresh", False):
        raise RuntimeError("operational profile must keep Safety fail-closed")
    if readiness.get("required_motion_backend", False) and not include_simulation:
        raise RuntimeError(
            "mission-capable operational profile requires the external Isaac/MoveIt composition"
        )

    actions = [
        SetEnvironmentVariable("SFTWIN_PROJECT_ROOT", str(artifact_directory.parent)),
        SetEnvironmentVariable(
            "RCUTILS_CONSOLE_OUTPUT_FORMAT",
            "[{severity}] [{name}] {message}",
        ),
        SetEnvironmentVariable("TZ", "UTC"),
        LogInfo(
            msg=(
                f"ARM Cell observation settings: log_level={selected_log_level} "
                f"MoveIt Planning Scene logger={selected_planning_scene_log_level} "
                f"diagnostics={','.join(sorted(selected_diagnostics)) if selected_diagnostics else 'none'}; "
                f"Vision capture={'enabled' if 'vision' in selected_diagnostics else 'disabled'}, "
                f"Motion Clock/Watchdog capture={'enabled' if 'motion' in selected_diagnostics else 'disabled'}, "
                f"Motion profile trace capture={'enabled' if 'motion' in selected_diagnostics else 'disabled'}, "
                f"Motion planning/execution detail loggers={'enabled' if selected_log_level == 'debug' else 'disabled'}, "
                f"Safety detail logger={'enabled' if selected_log_level == 'debug' else 'disabled'}, "
                f"Orchestration detail logger={'enabled' if selected_log_level == 'debug' else 'disabled'}, "
                "Isaac Gripper baseline capture is always enabled; detailed PhysX selection is session-local, "
                f"ROS session association={'joined' if live_session_root else 'unassociated'}, "
                f"artifact_root={artifact_directory}"
            )
        ),
        Node(
            package="arm_cell_vda",
            executable="external_state_mock_node",
            name="external_state_mock",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            ros_arguments=node_ros_arguments("arm_cell_vda", selected_log_level),
            parameters=[profile_parameters(config, "vda")],
        ),
        Node(
            package="arm_cell_gateway",
            executable="material_handoff_node",
            name="material_handoff_node",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            ros_arguments=node_ros_arguments("arm_cell_gateway", selected_log_level),
            parameters=[profile_parameters(config, "material_handoff")],
        ),
        Node(
            package="arm_cell_safety",
            executable="safety_node",
            name="safety_node",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            ros_arguments=node_ros_arguments(
                "arm_cell_safety",
                selected_log_level,
            ),
            parameters=[
                {
                    **profile_parameters(
                        {"use_sim_time": use_sim_time, "safety": config["safety"]},
                        "safety",
                    ),
                    "diagnostics.capture_enabled": "safety" in selected_diagnostics,
                    "diagnostics.output_directory": str(safety_artifact_directory),
                },
            ],
        ),
        Node(
            package="arm_cell_vision",
            executable="detect_target_node",
            name="detect_target_node",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            ros_arguments=node_ros_arguments("arm_cell_vision", selected_log_level),
            parameters=[
                {
                    **vision_profile_parameters(config, profile_path),
                    "diagnostics.capture_enabled": "vision" in selected_diagnostics,
                    "diagnostics.output_directory": str(vision_artifact_directory),
                    "diagnostics.live_test_session_id": live_session_id or "",
                    "diagnostics.snapshot_run_directory": str(
                        snapshot_run_directory or ""
                    ),
                }
            ],
        ),
        Node(
            package="arm_cell_task_execution",
            executable="task_execution_node",
            name="motion_node",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            ros_arguments=node_ros_arguments(
                "arm_cell_motion",
                selected_log_level,
                selected_action_logger_level,
                selected_planning_scene_log_level,
            ),
            parameters=[
                {
                    **motion_parameters,
                    "observation_reference": observation_reference,
                    "diagnostics.capture_enabled": "motion" in selected_diagnostics,
                    "diagnostics.output_directory": str(motion_artifact_directory),
                    "diagnostics.snapshot_run_directory": str(
                        snapshot_run_directory or artifact_directory / "snapshots"
                    ),
                    "home_joint_positions": home_joint_positions,
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                    **kinematics_parameters,
                }
            ],
        ),
        Node(
            package="arm_cell_orchestration",
            executable="orchestration_node",
            name="orchestration_node",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            ros_arguments=node_ros_arguments(
                "arm_cell_orchestration",
                selected_log_level,
                selected_action_logger_level,
            ),
            parameters=[
                {
                    **profile_parameters(config, "orchestration"),
                    "diagnostics.capture_enabled": "orchestration"
                    in selected_diagnostics,
                    "diagnostics.output_directory": str(
                        orchestration_artifact_directory
                    ),
                    "recipe_directory": str(
                        share
                        / "config"
                        / config["orchestration"].get("recipe_directory", "recipes")
                    ),
                }
            ],
        ),
    ]
    if live_session_root and not os.environ.get("ARM_CELL_PICK_EXECUTION_RECORD_DIR"):
        actions.insert(
            1,
            SetEnvironmentVariable(
                "ARM_CELL_PICK_EXECUTION_RECORD_DIR",
                str(live_session_root / "motion" / "pick_execution_records"),
            ),
        )

    if include_simulation:
        planning_profile = config.get("simulation", {}).get("planning_profile")
        if not planning_profile:
            raise RuntimeError("operational profile has no simulation planning profile")
        planning_path = Path(planning_profile)
        if not planning_path.is_absolute():
            planning_path = share / "config" / planning_path
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch/isaac_planning.launch.py")
                ),
                launch_arguments={
                    "profile": str(planning_path),
                    "static_scene": str(
                        config.get("simulation", {}).get("static_scene", False)
                    ).lower(),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch/isaac_camera.launch.py")
                )
            )
        )

    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
            ),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            DeclareLaunchArgument(
                "profile", default_value=str(share / "config/operational_profile.yaml")
            ),
            DeclareLaunchArgument("include_simulation", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "planning_scene_log_level",
                default_value="info",
                description=(
                    "MoveIt Planning Scene logger level; set to debug to inspect "
                    "temporary Scene construction during diagnostics"
                ),
            ),
            DeclareLaunchArgument(
                "diagnostics",
                default_value=DIAGNOSTICS_DEFAULT,
                description=(
                    "Optional component diagnostics; when omitted, debug selects all "
                    "ROS Bringup producers and other log levels select none"
                ),
            ),
            DeclareLaunchArgument("action_logger_level", default_value="warn"),
            OpaqueFunction(function=compose),
        ]
    )
