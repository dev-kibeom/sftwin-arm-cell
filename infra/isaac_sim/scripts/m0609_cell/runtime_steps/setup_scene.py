"""Script Editor-safe entrypoint for the M0609 cell scene composition."""

import atexit
import logging
import os
import sys
import math
from pathlib import Path

import yaml
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

try:
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.prims import delete_prim
    from isaacsim.core.utils.stage import add_reference_to_stage
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.utils.extensions import enable_extension
    from omni.isaac.core.utils.prims import delete_prim
    from omni.isaac.core.utils.stage import add_reference_to_stage

try:
    from isaacsim.storage.native import get_assets_root_path
except ImportError:
    try:
        from isaacsim.core.utils.nucleus import get_assets_root_path
    except ImportError:
        from omni.isaac.core.utils.nucleus import get_assets_root_path


def _add_cell_module_root():
    """Make sibling builders importable from a temporary Script Editor copy."""
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if configured_root:
        module_root = (
            Path(configured_root).expanduser()
            / "infra"
            / "isaac_sim"
            / "scripts"
            / "m0609_cell"
        )
    else:
        module_root = Path(__file__).resolve().parent
    if not (module_root / "scene_builder").is_dir():
        raise RuntimeError(
            "Cannot locate M0609 scene builders. When running from Isaac Script "
            "Editor, set SFTWIN_PROJECT_ROOT to the repository root."
        )
    if str(module_root) not in sys.path:
        sys.path.insert(0, str(module_root))


_add_cell_module_root()

from scene_builder.composition import run_scene_build
from scene_builder.material_supply import MaterialSupplyProfile
from scene_builder.material_supply_adapter import IsaacMaterialSupplyAdapter
from scene_builder.material_supply_sidecar import MaterialSupplySidecarClient

LOGGER = logging.getLogger(__name__)


ARM_JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
)


def _load_home_joint_positions():
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not configured_root:
        raise RuntimeError(
            "SFTWIN_PROJECT_ROOT is not configured. Run 1_before_play.py from "
            "the intended checkout."
        )
    home_path = (
        Path(configured_root).expanduser().resolve()
        / "ros2_ws/src/arm_cell_bringup/config/home_joint_positions.yaml"
    )
    config = yaml.safe_load(home_path.read_text()) or {}
    positions = tuple(float(value) for value in config.get("home_joint_positions", ()))
    if len(positions) != len(ARM_JOINT_NAMES) or not all(
        math.isfinite(value) for value in positions
    ):
        raise RuntimeError(
            "canonical HOME pose must contain six finite joint positions"
        )
    return positions


def _load_operational_profile():
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not configured_root:
        raise RuntimeError(
            "SFTWIN_PROJECT_ROOT is not configured. Run 1_before_play.py from "
            "the intended checkout."
        )
    profile_path = (
        Path(configured_root).expanduser().resolve()
        / "ros2_ws/src/arm_cell_bringup/config/operational_profile.yaml"
    )
    return yaml.safe_load(profile_path.read_text()) or {}


def _author_initial_home_targets(stage):
    """Set Play-time angular drive targets from the canonical HOME pose.

    USD angular drive target positions are authored in degrees.  The canonical
    repository HOME contract remains radians, so conversion is kept explicit
    at this Isaac scene boundary.
    """
    home_positions = _load_home_joint_positions()
    joints = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.IsValid() and prim.GetName() in ARM_JOINT_NAMES
    }
    missing = [name for name in ARM_JOINT_NAMES if name not in joints]
    if missing:
        raise RuntimeError(f"HOME joints not found in robot stage: {missing}")

    for name, position_rad in zip(ARM_JOINT_NAMES, home_positions):
        drive = UsdPhysics.DriveAPI.Get(joints[name], "angular")
        if not drive:
            raise RuntimeError(f"angular drive not found for HOME joint: {name}")
        target = drive.GetTargetPositionAttr()
        if not target.IsValid():
            raise RuntimeError(
                f"angular drive target unavailable for HOME joint: {name}"
            )
        target.Set(math.degrees(position_rad))
        joints[name].SetCustomDataByKey(
            "sf_twin:initial_home_position_rad", position_rad
        )


def _apply_joint_2_stiffness_profile(stage):
    """Apply the external W06 production scale to the imported joint_2 drive."""
    simulation = _load_operational_profile().get("simulation", {})
    scale = float(simulation.get("joint_2_stiffness_scale", 1.0))
    if not math.isfinite(scale) or scale <= 0.0:
        raise RuntimeError(
            "simulation.joint_2_stiffness_scale must be a finite positive value"
        )

    joints = [
        prim
        for prim in stage.Traverse()
        if prim.IsValid() and prim.GetName() == "joint_2"
    ]
    if len(joints) != 1:
        raise RuntimeError(
            f"expected exactly one joint_2 in robot stage, found {len(joints)}"
        )
    drive = UsdPhysics.DriveAPI.Get(joints[0], "angular")
    if not drive:
        raise RuntimeError("angular drive not found for joint_2")
    stiffness = drive.GetStiffnessAttr()
    if not stiffness.IsValid():
        raise RuntimeError("angular drive stiffness unavailable for joint_2")
    base_stiffness = float(stiffness.Get())
    if not math.isfinite(base_stiffness) or base_stiffness <= 0.0:
        raise RuntimeError("joint_2 imported angular stiffness must be positive")
    stiffness.Set(base_stiffness * scale)


_material_supply_adapter = None
_material_supply_update_subscription = None
_material_supply_timeline_subscription = None
_live_test_session = None


def _start_live_test_session():
    global _live_test_session
    if _live_test_session is not None:
        return
    try:
        from shared.live_test_session import LiveTestSession

        from shared.environment_setup import resolve_diagnostics_selection

        selected = resolve_diagnostics_selection()
        _live_test_session = LiveTestSession(
            Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()
            / ".local_artifacts",
            selected,
        )
        os.environ["SFTWIN_LIVE_TEST_SESSION_PATH"] = str(_live_test_session.descriptor)
        if _material_supply_adapter is not None:
            _material_supply_adapter.set_live_test_session_id(
                _live_test_session.session_id
            )
    except Exception as error:
        _live_test_session = None
        print(f"Live test session recording could not be started: {error}")


def _finish_live_test_session():
    global _live_test_session
    if _live_test_session is None:
        return
    session = _live_test_session
    _live_test_session = None
    if _material_supply_adapter is not None:
        _material_supply_adapter.set_live_test_session_id(None)
    os.environ.pop("SFTWIN_LIVE_TEST_SESSION_PATH", None)
    try:
        descriptor = session.finish()
    except Exception as error:
        print(f"Live test session recording could not be finalized: {error}")
        return
    if descriptor is None:
        return


def _spin_material_supply_while_playing():
    if _material_supply_adapter is None:
        return
    import omni.timeline

    if omni.timeline.get_timeline_interface().is_playing():
        _material_supply_adapter.spin_once()


def _on_material_supply_timeline_event(event):
    import omni.timeline

    event_type = getattr(event, "type", None)
    try:
        is_stopped = int(event_type) == int(omni.timeline.TimelineEventType.STOP)
    except (TypeError, ValueError):
        is_stopped = event_type == omni.timeline.TimelineEventType.STOP
    is_playing = False
    try:
        is_playing = int(event_type) == int(omni.timeline.TimelineEventType.PLAY)
    except (TypeError, ValueError, AttributeError):
        is_playing = event_type == getattr(
            omni.timeline.TimelineEventType, "PLAY", object()
        )
    if is_playing:
        _start_live_test_session()
    if is_stopped:
        reset_gripper = getattr(
            globals().get("runtime_session"), "reset_for_simulation_stop", None
        )
        gripper_reset = True
        if callable(reset_gripper):
            try:
                gripper_reset = bool(reset_gripper())
            except Exception as error:
                gripper_reset = False
                print(f"Simulation Stop could not reset the gripper: {error}")
        close_writer = getattr(
            globals().get("runtime_session"), "close_diagnostic_writer", None
        )
        if callable(close_writer):
            try:
                close_writer()
            except Exception as error:
                print(f"Simulation Stop could not close diagnostic writer: {error}")
        try:
            _author_initial_home_targets(World.instance().stage)
        except Exception as error:
            gripper_reset = False
            print(f"Simulation Stop could not restore the HOME pose: {error}")
        if _material_supply_adapter is not None:
            result = _material_supply_adapter.reset_generated_materials(
                attachment_reset=lambda: gripper_reset
            )
            if result:
                print("Simulation Stop reset completed.")
        _finish_live_test_session()


def _close_material_supply_adapter():
    global _material_supply_adapter
    global _material_supply_update_subscription
    global _material_supply_timeline_subscription

    if _material_supply_update_subscription is not None:
        _material_supply_update_subscription.unsubscribe()
        _material_supply_update_subscription = None
    if _material_supply_timeline_subscription is not None:
        _material_supply_timeline_subscription.unsubscribe()
        _material_supply_timeline_subscription = None
    if _material_supply_adapter is not None:
        _material_supply_adapter.close()
        _material_supply_adapter = None


atexit.register(_close_material_supply_adapter)


def _start_material_supply_adapter(context):
    """Connect the Isaac supply realization to Integration's accepted event."""
    global _material_supply_adapter
    global _material_supply_update_subscription
    global _material_supply_timeline_subscription

    _close_material_supply_adapter()

    project_root = Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()
    supply_config_path = project_root / "infra/isaac_sim/config/material_supply.yaml"
    if not supply_config_path.is_file():
        raise RuntimeError(
            f"material supply configuration is unavailable: {supply_config_path}"
        )
    supply_config = yaml.safe_load(supply_config_path.read_text()) or {}
    operational_profile = _load_operational_profile()
    material_handoff = operational_profile.get("material_handoff", {})
    profile = MaterialSupplyProfile.from_mapping(
        supply_config,
        operational_profile.get("vision", {}),
        operational_profile.get("material_definitions", {}),
    )
    sidecar = MaterialSupplySidecarClient.launch(project_root)
    try:
        import omni.timeline

        _material_supply_adapter = IsaacMaterialSupplyAdapter(
            stage=World.instance().stage,
            sidecar=sidecar,
            profile=profile,
            delivery_deadline_ms=material_handoff.get("delivery_deadline_ms"),
            shelf_origin_m=(
                context.tray_x,
                context.tray_y,
                context.amr_tray_z + 0.0225,
            ),
            artifact_directory=project_root / ".local_artifacts/isaac_pose_recordings",
            timeline=omni.timeline.get_timeline_interface(),
        )
    except Exception:
        sidecar.close()
        raise
    try:
        import omni.kit.app
        import omni.timeline

        _material_supply_update_subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                lambda _event: _spin_material_supply_while_playing(),
                name="arm_cell_isaac_material_supply_adapter",
            )
        )
        _material_supply_timeline_subscription = (
            omni.timeline.get_timeline_interface()
            .get_timeline_event_stream()
            .create_subscription_to_pop(
                _on_material_supply_timeline_event,
                name="arm_cell_isaac_timeline_session_and_stop_cleanup",
            )
        )
        if omni.timeline.get_timeline_interface().is_playing():
            _start_live_test_session()
    except Exception:
        _close_material_supply_adapter()
        raise


context = run_scene_build(
    world_cls=World,
    enable_extension=enable_extension,
    delete_prim=delete_prim,
    get_assets_root_path=get_assets_root_path,
    add_reference_to_stage=add_reference_to_stage,
    gf=Gf,
    usd_geom=UsdGeom,
    usd_lux=UsdLux,
    script_path=__file__,
)

_author_initial_home_targets(World.instance().stage)
_apply_joint_2_stiffness_profile(World.instance().stage)
_start_material_supply_adapter(context)
