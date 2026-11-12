"""Compose the supported software-only ARM Cell operational profile."""

import fcntl
import hashlib
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import uuid

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.logging import launch_config
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import xacro
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


def project_checkout_root():
    """Locate the checkout root so diagnostics do not depend on process cwd."""
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    candidates = [Path(configured_root).expanduser()] if configured_root else []
    candidates.extend((Path.cwd(), Path(__file__).resolve()))
    for candidate in candidates:
        candidate = candidate.resolve()
        for root in (candidate, *candidate.parents):
            if (root / "ros2_ws/src").is_dir() and (
                root / "infra/isaac_sim/scripts/m0609_cell"
            ).is_dir():
                return root
            if (root / "src").is_dir() and (
                root.parent / "infra/isaac_sim/scripts/m0609_cell"
            ).is_dir():
                return root.parent
    return Path.cwd()


def project_artifacts_directory():
    root = project_checkout_root()
    config_path = root / "config/run_records.yaml"
    config = (yaml.safe_load(config_path.read_text()) or {}) if config_path.is_file() else {}
    return str((root / Path(config.get("root", ".run_records")).expanduser()).resolve())


def record_directory_name(kind):
    config_path = project_checkout_root() / "config/run_records.yaml"
    config = (yaml.safe_load(config_path.read_text()) or {}) if config_path.is_file() else {}
    return config.get("directories", {}).get(kind, kind)


def diagnostic_root(artifact_directory):
    return Path(artifact_directory) / record_directory_name("diagnostic")


def _git_provenance():
    """Capture source checkout revision, not the running binary revision."""
    result = {
        "source": "git worktree",
        "source_root": None,
        "revision": None,
        "dirty_state": "unavailable",
        "unavailable_reason": None,
    }
    project_root = Path(
        os.environ.get("SFTWIN_PROJECT_ROOT", project_checkout_root())
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


def _live_isaac_descriptors(artifact_directory):
    """Join only one explicit open Isaac descriptor; never guess by recency."""
    root = Path(artifact_directory)
    descriptors = []
    unreadable = False
    if root.is_dir():
        diagnostic = record_directory_name("diagnostic")
        for descriptor in set((
            *root.glob(f"[0-9]*/[0-9]*/{diagnostic}/session.json"),
            *root.glob("[0-9]*/[0-9]*/diagnostic/session.json"),
            *root.glob("[0-9]*/[0-9]*/diagnostic_records/session.json"),
            *root.glob("diagnostic_records/[0-9]*/**/session.json"),
        )):
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
        return []
    return descriptors


def join_live_test_session(artifact_directory):
    """Join a unique live Isaac session only once per Vision namespace."""
    descriptors = _live_isaac_descriptors(artifact_directory)
    if not descriptors:
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
        "git_provenance": _git_provenance(),
    }
    recording_locations = {
        "ros_logs": str(launch_config.log_dir),
        "clock": str(descriptor.parent / "clock"),
        "simulation": str(descriptor.parent / "simulation"),
        "snapshots": str(
            reserve_snapshot_run_directory(artifact_directory, descriptor.parent)
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
            # Recheck under the descriptor lock: separate Bringups cannot
            # share Acquisition Generation numbers after a Vision restart.
            if any(p.get("process") == "ros_bringup" and p.get("instance_id") != instance_id
                   for p in participants if isinstance(p, dict)):
                return None
            if not any(item.get("instance_id") == instance_id for item in participants):
                participants.append(participant)
            # These are writer destinations, not a claim that files already
            # exist or that shared snapshot directories belong to this session.
            latest["session_association"] = "associated"
            latest.setdefault("camera_configuration", camera_configuration_provenance())
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


def camera_configuration_provenance():
    """Identify the default calibration input without changing camera behavior."""
    try:
        path = (Path(get_package_share_directory("arm_cell_sim_adapter")) /
                "config/isaac_camera_stage_snapshot.json")
        return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "scope": "packaged_default"}
    except (OSError, LookupError):
        return {"path": None, "sha256": None, "scope": "packaged_default", "status": "unavailable"}


def remove_empty_run(run_root):
    """Remove only empty directories beneath an exclusively reserved run."""
    run_root = Path(run_root)
    for directory in sorted(run_root.rglob("*"), key=lambda p: len(p.parts), reverse=True):
        if directory.is_dir() and not directory.is_symlink():
            try:
                directory.rmdir()
            except OSError:
                pass
    try:
        run_root.rmdir()
    except OSError:
        pass


def create_recording_session(artifact_directory, runtime, related_session_id=None, related_session_root=None):
    """Create a ROS-owned descriptor when no Isaac session can be joined."""
    now = datetime.now().astimezone()
    date_root = Path(artifact_directory) / now.strftime("%Y%m%d")
    root = None
    run_root = None
    temporary = None
    try:
        date_root.mkdir(parents=True, exist_ok=True)
        suffix = 0
        while True:
            name = now.strftime("%H%M%S") + (f"-{suffix:02d}" if suffix else "")
            run_root = date_root / name
            try:
                run_root.mkdir()
                break
            except FileExistsError:
                suffix += 1
        root = diagnostic_root(run_root)
        root.mkdir(parents=True)
        data = {
            "schema_version": 1,
            "session_id": str(uuid.uuid4()),
            "session_association": "unassociated",
            "status": "open",
            "owner": "ros_bringup",
            "owner_pid": os.getpid(),
            "related_live_test_session_id": related_session_id,
            "runtime": runtime,
            "camera_configuration": camera_configuration_provenance(),
            "started_at": now.isoformat(),
            "timezone": str(now.tzinfo),
            "ended_at": None,
            "end_condition": None,
            "participants": [{"process": "ros_bringup", "instance_id": str(os.getpid()),
                              "status": "participating",
                              "git_provenance": _git_provenance()}],
            "recording_locations": {
                name: str(root / name)
                for name in ("vision", "motion", "gripper", "safety", "orchestration", "clock", "simulation")
            },
        }
        if related_session_root is not None:
            related_root = Path(related_session_root)
            related_data = json.loads((related_root / "session.json").read_text(encoding="utf-8"))
            if related_data.get("session_id") != related_session_id:
                raise ValueError("related Isaac session changed")
            data["related_live_test_session_path"] = str(related_root / "session.json")
            for component in ("gripper", "simulation"):
                source = Path(related_data.get("recording_locations", {}).get(component, component))
                if not source.is_absolute():
                    source = related_root / source
                data["recording_locations"][component] = str(source)
        data["recording_locations"]["ros_logs"] = str(launch_config.log_dir)
        data["recording_locations"]["snapshots"] = str(
            run_root / record_directory_name("snapshots"))
        descriptor = root / "session.json"
        temporary = descriptor.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, descriptor)
        return root
    except (OSError, ValueError):
        try:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
            if run_root is not None:
                remove_empty_run(run_root)
        except OSError:
            pass
        return None


def record_session_configuration(root, config, profile_path, selection, include_simulation):
    """Index selected inputs; recording failures never affect runtime results."""
    descriptor = Path(root) / "session.json"
    calibration = Path(vision_profile_parameters(config, profile_path)["calibration_file"])
    try:
        calibration_hash = hashlib.sha256(calibration.read_bytes()).hexdigest()
    except OSError:
        calibration_hash = None
    vision = config.get("vision", {})
    camera_topic = vision.get("camera_info_topic", "")
    camera_id = vision.get("camera_id")
    selection_record = {
        "diagnostic_selection": sorted(selection),
        "simulation_composition_enabled": include_simulation,
        "camera": {
            "camera_id": camera_id,
            "identity_status": "configured" if camera_id else "not_configured",
            "camera_info_topic": camera_topic,
            "rgb_topic": vision.get("rgb_topic"),
            "depth_topic": vision.get("depth_topic"),
            "vision_calibration": {"path": str(calibration), "sha256": calibration_hash,
                                   "status": "available" if calibration_hash else "unavailable"},
        },
    }
    try:
        with Path(str(descriptor) + ".lock").open("a+") as lock_stream:
            fcntl.flock(lock_stream, fcntl.LOCK_EX)
            data = json.loads(descriptor.read_text(encoding="utf-8"))
            if data.get("status") != "open":
                return
            data.update(selection_record)
            temporary = descriptor.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
            os.replace(temporary, descriptor)
    except (OSError, ValueError, AttributeError):
        pass


def finish_recording_session(context, root):
    """Only a clean launch shutdown closes a ROS-owned recording session."""
    descriptor = Path(root) / "session.json"
    try:
        with Path(str(descriptor) + ".lock").open("a+") as lock_stream:
            fcntl.flock(lock_stream, fcntl.LOCK_EX)
            data = json.loads(descriptor.read_text(encoding="utf-8"))
            if (data.get("owner") != "ros_bringup" or data.get("status") != "open"
                    or data.get("owner_pid") != os.getpid()):
                return []
            data.update(status="closed", ended_at=datetime.now().astimezone().isoformat(),
                        end_condition="bringup_shutdown")
            for participant in data.get("participants", []):
                participant["status"] = "session_ended"
            temporary = descriptor.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
            os.replace(temporary, descriptor)
    except (OSError, ValueError, AttributeError):
        pass
    return []


def reserve_snapshot_run_directory(artifact_directory, session_root=None):
    """Choose the session's shared destination; writers create it on first save."""
    if session_root is not None:
        session_root = Path(session_root)
        data = json.loads((session_root / "session.json").read_text(encoding="utf-8"))
        recorded = data.get("artifact_locations", {}).get("vision_snapshots", {}).get("path")
        recorded = recorded or data.get("recording_locations", {}).get("snapshots")
        if recorded:
            path = Path(recorded)
            return path if path.is_absolute() else session_root / path
        diagnostic_parts = Path(record_directory_name("diagnostic")).parts
        if tuple(session_root.parts[-len(diagnostic_parts):]) == diagnostic_parts:
            run_root = session_root
            for _ in diagnostic_parts:
                run_root = run_root.parent
            return run_root / record_directory_name("snapshots")
        if session_root.name in ("diagnostic", "diagnostic_records"):
            return session_root.parent / record_directory_name("snapshots")
        # Existing descriptors keep their original artifact locations.
        return (Path(artifact_directory) / record_directory_name("snapshots") /
                Path(session_root).parent.name / Path(session_root).name)
    now = datetime.now().astimezone()
    return (Path(artifact_directory) / now.strftime("%Y%m%d") /
            f"{now:%H%M%S}-{now:%f}-{os.getpid()}" / record_directory_name("snapshots"))


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
        # These independent MoveIt namespaces otherwise inherit global DEBUG.
        # Reuse the existing detail override; retain their WARN/ERROR output.
        for logger_name in (PLANNING_SCENE_LOGGER, "moveit_robot_state.conversions",
                            "moveit_robot_state.robot_state"):
            arguments.extend(["--log-level", f"{logger_name}:={planning_scene_log_level}"])
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
    owns_recording_session = live_session_root is None
    related_session_root = live_session_root
    if live_session_root is None:
        runtime = ("simulation" if LaunchConfiguration("include_simulation").perform(context).lower() == "true"
                   else "unspecified")
        related = _live_isaac_descriptors(artifact_directory)
        related_id = related[0][1]["session_id"] if len(related) == 1 else None
        related_session_root = related[0][0].parent if len(related) == 1 else None
        live_session_root = create_recording_session(artifact_directory, runtime, related_id, related_session_root)
    live_session_id = None
    gripper_record_directory = None
    if live_session_root:
        try:
            session_data = json.loads(
                (live_session_root / "session.json").read_text(encoding="utf-8")
            )
            live_session_id = session_data.get("session_id")
            gripper_record_directory = Path(
                session_data.get("recording_locations", {}).get("gripper", "gripper"))
            if not gripper_record_directory.is_absolute():
                gripper_record_directory = live_session_root / gripper_record_directory
        except (OSError, ValueError, AttributeError):
            live_session_root = None
        if not live_session_id:
            live_session_root = None
    if live_session_root is None:
        # Reserve one fallback namespace for all writers in this launch.
        now = datetime.now().astimezone()
        date_root = artifact_directory / now.strftime("%Y%m%d")
        date_root.mkdir(parents=True, exist_ok=True)
        suffix = 0
        while True:
            fallback_run = date_root / (now.strftime("%H%M%S") + (f"-{suffix:02d}" if suffix else ""))
            try:
                fallback_run.mkdir()
                break
            except FileExistsError:
                suffix += 1
        artifact_directory = fallback_run
        fallback_temporary = None
        try:
            fallback_diagnostic = diagnostic_root(fallback_run)
            fallback_diagnostic.mkdir(parents=True)
            fallback_temporary = fallback_diagnostic / "session.json.tmp"
            fallback_temporary.write_text(json.dumps({
                "schema_version": 1, "session_id": None,
                "session_association": "unassociated", "owner_pid": os.getpid(),
                "started_at": now.isoformat(),
            }, indent=2) + "\n", encoding="utf-8")
            os.replace(fallback_temporary, fallback_diagnostic / "session.json")
        except OSError:
            try:
                if fallback_temporary is not None:
                    fallback_temporary.unlink(missing_ok=True)
            except OSError:
                pass
            remove_empty_run(fallback_run)
            # Writers may recreate their destinations; metadata is best effort.
    vision_artifact_directory = (
        live_session_root / "vision"
        if live_session_root
        else diagnostic_root(artifact_directory) / "vision"
    )
    snapshot_run_directory = (reserve_snapshot_run_directory(artifact_directory, live_session_root)
                              if live_session_root else artifact_directory / record_directory_name("snapshots"))
    motion_artifact_directory = (
        live_session_root / "motion"
        if live_session_root
        else diagnostic_root(artifact_directory) / "motion"
    )
    safety_artifact_directory = (
        live_session_root / "safety"
        if live_session_root
        else diagnostic_root(artifact_directory) / "safety"
    )
    orchestration_artifact_directory = (
        live_session_root / "orchestration"
        if live_session_root
        else diagnostic_root(artifact_directory) / "orchestration"
    )
    selected_action_logger_level = LaunchConfiguration("action_logger_level").perform(
        context
    )
    profile_path = Path(LaunchConfiguration("profile").perform(context))
    config = load_profile(profile_path)
    material_handoff_config = dict(config.get("material_handoff", {}))
    motion_parameters = profile_parameters(config, "motion")
    motion_share = Path(get_package_share_directory("arm_cell_motion"))
    waypoint_config = yaml.safe_load(
        (motion_share / "config/place_waypoint.yaml").read_text()
    )
    configured_trajectory = (
        motion_parameters.get("place_waypoint_trajectory_file")
        or waypoint_config["place_waypoint_trajectory_file"]
    )
    trajectory_file = Path(configured_trajectory).expanduser()
    if not trajectory_file.is_absolute():
        trajectory_file = project_checkout_root() / trajectory_file
    motion_parameters["place_waypoint_trajectory_file"] = str(trajectory_file.resolve())
    share = Path(get_package_share_directory("arm_cell_bringup"))
    home_config = (
        yaml.safe_load((motion_share / "config/home_joint_positions.yaml").read_text()) or {}
    )
    home_joint_positions = home_config["home_joint_positions"]
    if len(home_joint_positions) != 6:
        raise RuntimeError("home_joint_positions must contain six arm joints")
    use_sim_time = config.get("use_sim_time", True)
    description = motion_share / "assets/robots/m0609_robotiq_2f85"
    moveit_share = motion_share
    robot_description = xacro.process_file(
        str(description / "urdf/m0609_robotiq_2f85.urdf.xacro"),
        mappings={
            "mesh_root": (
                "package://arm_cell_motion/assets/robots/"
                "m0609_robotiq_2f85/meshes"
            )
        },
    ).toxml()
    robot_description_semantic = (
        moveit_share / "config/moveit/m0609_robotiq_2f85.srdf"
    ).read_text()
    kinematics = yaml.safe_load((moveit_share / "config/moveit/kinematics.yaml").read_text())
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
    if live_session_root:
        record_session_configuration(live_session_root, config, profile_path,
                                     selected_diagnostics, include_simulation)
    if not readiness.get("fail_closed_until_inputs_fresh", False):
        raise RuntimeError("operational profile must keep Safety fail-closed")
    if readiness.get("required_motion_backend", False) and not include_simulation:
        raise RuntimeError(
            "mission-capable operational profile requires the external Isaac/MoveIt composition"
        )

    actions = [
        SetEnvironmentVariable("SFTWIN_PROJECT_ROOT", str(project_checkout_root())),
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
                f"ROS session association={'created' if owns_recording_session and live_session_root else 'joined' if live_session_root else 'unassociated'}, "
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
                        snapshot_run_directory or artifact_directory / record_directory_name("snapshots")
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
    if live_session_root:
        actions.insert(1, SetEnvironmentVariable(
            "ARM_CELL_RECORDING_SESSION_PATH", str(live_session_root / "session.json")))
        actions.insert(1, SetEnvironmentVariable(
            "ARM_CELL_GRIPPER_RECORD_DIR", str(gripper_record_directory)))
    if live_session_root and owns_recording_session:
        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=finish_recording_session,
                                        args=[live_session_root])]
        )))
    if not os.environ.get("ARM_CELL_PICK_EXECUTION_RECORD_DIR"):
        actions.insert(
            1,
            SetEnvironmentVariable(
                "ARM_CELL_PICK_EXECUTION_RECORD_DIR",
                str(motion_artifact_directory / "pick_execution_records"),
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
                "profile", default_value=os.environ.get("ARM_CELL_PROFILE", str(share / "config/operational_profile.yaml"))
            ),
            DeclareLaunchArgument("include_simulation", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "planning_scene_log_level",
                default_value="info",
                description=(
                    "MoveIt Planning Scene and RobotState logger level; set to debug to inspect "
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
