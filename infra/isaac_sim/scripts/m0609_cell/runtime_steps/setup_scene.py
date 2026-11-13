"""Script Editor-safe entrypoint for the M0609 cell scene composition."""

import atexit
import logging
import os
import sys
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

# Scene discovery uses only the common scripts root. The Cell path above is
# still needed by the unchanged HOME/drive/material-supply runtime below.
scripts_root = Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve() / "infra/isaac_sim/scripts"
if str(scripts_root) not in sys.path:
    sys.path.insert(0, str(scripts_root))
from scene_entrypoint import run_scene_build
from scene_builder.material_supply import MaterialSupplyProfile
from scene_builder.material_supply_adapter import IsaacMaterialSupplyAdapter
from scene_builder.material_supply_sidecar import MaterialSupplySidecarClient

LOGGER = logging.getLogger(__name__)


def _load_operational_profile():
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not configured_root:
        raise RuntimeError(
            "SFTWIN_PROJECT_ROOT is not configured. Run 1_before_play.py from "
            "the intended checkout."
        )
    from runtime_configuration import load_runtime_configuration
    return load_runtime_configuration()



# Keep one selected binding for initial setup and the Stop callback.
from robot_binding import load_robot_binding, author_home_targets, apply_joint_drive_profile

_robot_binding = load_robot_binding()


def _author_initial_home_targets(stage):
    author_home_targets(stage, _robot_binding, UsdPhysics)


_material_supply_adapter = None
_material_supply_update_subscription = None
_material_supply_timeline_subscription = None
_live_test_session = None


def _start_live_test_session():
    global _live_test_session
    if _live_test_session is not None:
        return
    try:
        from shared.run_records import run_records_root
        from shared.live_test_session import LiveTestSession

        from shared.environment_setup import resolve_diagnostics_selection

        selected = resolve_diagnostics_selection()
        _live_test_session = LiveTestSession(
            run_records_root(Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()),
            selected,
        )
        os.environ["SFTWIN_LIVE_TEST_SESSION_PATH"] = str(_live_test_session.descriptor)
        if _material_supply_adapter is not None:
            _material_supply_adapter.set_live_test_session_id(
                _live_test_session.session_id, _live_test_session.root
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
    from shared.run_records import diagnostic_component_directory
    global _material_supply_adapter
    global _material_supply_update_subscription
    global _material_supply_timeline_subscription

    _close_material_supply_adapter()

    project_root = Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()
    ros_install = project_root / "ros2_ws/install/setup.bash"
    if not ros_install.is_file():
        print(
            "ROS workspace is not built locally; waiting for the Docker "
            "material supply sidecar."
        )
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
    from copy import deepcopy
    from dataclasses import replace
    material_profiles = {}
    for identity, material in operational_profile["material_catalog"].items():
        selected_supply = deepcopy(supply_config)
        selected_supply["material_supply"]["material_type"] = identity
        selected_supply["material_supply"]["mass_kg"] = material.get("mass_kg", profile.mass_kg)
        selected_profile = MaterialSupplyProfile.from_mapping(
            selected_supply, operational_profile["vision"],
            operational_profile["material_definitions"],
        )
        material_profiles[identity] = replace(selected_profile, display_color_rgb=tuple(material["display_color_rgb"]))
    sidecar = MaterialSupplySidecarClient.launch(project_root)
    try:
        import omni.timeline

        _material_supply_adapter = IsaacMaterialSupplyAdapter(
            stage=World.instance().stage,
            sidecar=sidecar,
            profile=profile,
            material_profiles=material_profiles,
            delivery_deadline_ms=material_handoff.get("delivery_deadline_ms"),
            shelf_origin_m=(
                context.tray_x,
                context.tray_y,
                context.amr_tray_z + 0.0225,
            ),
            artifact_directory=(
                diagnostic_component_directory(project_root, "simulation")
            ),
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
    configuration=_load_operational_profile(),
    world_cls=World,
    enable_extension=enable_extension,
    delete_prim=delete_prim,
    add_reference_to_stage=add_reference_to_stage,
    gf=Gf,
    usd_geom=UsdGeom,
    usd_lux=UsdLux,
    script_path=__file__,
)

_author_initial_home_targets(World.instance().stage)
apply_joint_drive_profile(World.instance().stage, _robot_binding, UsdPhysics)
_start_material_supply_adapter(context)
