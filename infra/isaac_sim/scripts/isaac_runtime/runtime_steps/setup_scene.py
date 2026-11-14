"""Scene, Robot binding and Timeline session orchestration for Isaac."""

import atexit
import os
from pathlib import Path

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


from scene_entrypoint import run_scene_build
from isaac_runtime.robot_binding import RobotBinding, author_home_targets, apply_joint_drive_profile
from isaac_runtime.runtime_workflow import call_selected


# Retain the same binding for initial HOME and Stop/Reset.
_robot_binding = RobotBinding.from_runtime(runtime_configuration)


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
        from isaac_runtime.run_records import run_records_root
        from isaac_runtime.live_test_session import LiveTestSession

        from isaac_runtime.environment_setup import resolve_diagnostics_selection

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


def close_runtime():
    owner = globals().get("pnp_validation_session_owner")
    session = globals().get("runtime_session")
    if owner is not None:
        owner.prepare_for_rerun()
    elif session is not None:
        session.shutdown()
    globals()["runtime_session"] = None
    globals()["pnp_validation_session_owner"] = None
    _finish_live_test_session()
    _close_material_supply_adapter()


# Re-execution replaces the old shutdown callback rather than accumulating it.
previous_close = globals().get("_registered_runtime_close")
if previous_close is not None:
    atexit.unregister(previous_close)
_registered_runtime_close = close_runtime
atexit.register(close_runtime)


def _start_material_supply_adapter(context):
    global _material_supply_adapter
    global _material_supply_update_subscription
    global _material_supply_timeline_subscription
    _close_material_supply_adapter()
    _material_supply_adapter = call_selected(
        runtime_configuration['cell']['isaac']['material_supply_factory'],
        configuration=runtime_configuration, context=context,
    )
    try:
        import omni.kit.app
        import omni.timeline
        _material_supply_update_subscription = (
            omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                lambda _event: _spin_material_supply_while_playing(),
                name="arm_cell_isaac_material_supply_adapter",
            )
        )
        _material_supply_timeline_subscription = (
            omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(
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
    configuration=runtime_configuration,
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
prepare_tool = runtime_configuration['tool']['runtime'].get('isaac_prepare')
if prepare_tool:
    call_selected(prepare_tool, configuration=runtime_configuration, stage=World.instance().stage)
_start_material_supply_adapter(context)
