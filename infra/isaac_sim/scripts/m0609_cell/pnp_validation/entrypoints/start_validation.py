"""Isaac Script Editor entrypoint for the deterministic PnP diagnostic fixture.

Run the role based stage setup, press Play, then run this file in Isaac Script Editor.
It bootstraps the diagnostic gripper profile and writes a runtime JSON handoff
for the ROS-side registration helper; it does not import or call ROS APIs.
"""

# The diagnostic is an isolated Fixed Vision/cube path. Rebootstrap only the
# gripper runtime into its diagnostic profile before creating its fixture.
import os as _os
from pathlib import Path as _Path

_diagnostic_root = _Path(_os.environ.get("SFTWIN_PROJECT_ROOT", "")).expanduser()
_diagnostic_file = __file__
_is_script_editor_copy = _Path(_diagnostic_file).name.startswith("script_") and any(
    parent.name.startswith("carb") for parent in _Path(_diagnostic_file).parents
)
if _is_script_editor_copy:
    if not _diagnostic_root.is_dir():
        raise RuntimeError("Run 1_before_play.py before the PnP diagnostic")
    _os.environ["SFTWIN_RUNTIME_MODE"] = "diagnostic"
    _bootstrap_path = (
        _diagnostic_root
        / "infra/isaac_sim/scripts/m0609_cell/runtime_steps/bootstrap_gripper.py"
    )
    exec(
        compile(
            _bootstrap_path.read_text(encoding="utf-8"), str(_bootstrap_path), "exec"
        ),
        globals(),
    )
    __file__ = _diagnostic_file

import os
from pathlib import Path
import time
import math
import sys
import importlib
import inspect
from types import MethodType

import yaml


_project_root = os.environ.get("SFTWIN_PROJECT_ROOT")
if not _project_root:
    raise RuntimeError("Set SFTWIN_PROJECT_ROOT to the extracted repository root")
_validation_root = (
    Path(_project_root).expanduser() / "infra/isaac_sim/scripts/m0609_cell"
)
if str(_validation_root) in sys.path:
    sys.path.remove(str(_validation_root))
sys.path.insert(0, str(_validation_root))
importlib.invalidate_caches()
import shared.script_editor_bootstrap as script_editor_bootstrap

# Script Editor keeps the graph builder imported by the ActionGraph build step
# alive across entrypoint executions. The diagnostic terminal-cleanup callback
# may run much later, so refresh the package before it captures
# build_ros_action_graph for graph recovery.
script_editor_bootstrap.clear_imported_package("graph_builder")
graph_builder_cache = _validation_root / "graph_builder" / "__pycache__"
if graph_builder_cache.is_dir():
    for cached_module in graph_builder_cache.glob("*.pyc"):
        cached_module.unlink(missing_ok=True)
importlib.invalidate_caches()

validation_cache = _validation_root / "pnp_validation" / "__pycache__"
if validation_cache.is_dir():
    for cached_module in validation_cache.glob("*.pyc"):
        cached_module.unlink(missing_ok=True)


def _fresh_import_validation_modules():
    """Drop only validation-private modules; keep Isaac stage/runtime intact."""
    for module_name in list(sys.modules):
        if module_name == "pnp_validation" or module_name.startswith("pnp_validation."):
            del sys.modules[module_name]
    import pnp_validation.live_fixture as live_fixture_module
    import pnp_validation.run_validation as run_validation_module

    return live_fixture_module, run_validation_module


# Isaac Script Editor reuses one Python process across executions.  A package
# purge is stronger than reload(): it also removes stale handoff/fixture
# submodules imported before the entrypoint.  It does not reset the Isaac stage.
_live_fixture_module, _run_validation_module = _fresh_import_validation_modules()
print(f"[PnP Diagnostic] validation module: {_run_validation_module.__file__}")
print(f"[PnP Diagnostic] fixture module: {_live_fixture_module.__file__}")

from pnp_validation.run_validation import (
    CleanupRequestWorker,
    MissionCompletionEventWorker,
    PlaceResultEventWorker,
    IsaacFixtureRuntime,
    ValidationRun,
    load_validation_profile,
    request_validation_start_state,
    validation_start_state_ready,
    validation_start_diagnostic,
    ValidationStartPending,
    ValidationStartRejected,
    fixture_profile_from_validation_config,
)
from pnp_validation.handoff import (
    RegistrationHandoff,
    atomic_write_registration_handoff,
)


def configure_validation_environment(environ=None):
    """Set only run-specific defaults; common profile state belongs to bootstrap."""
    environment = os.environ if environ is None else environ
    if not environment.get("SFTWIN_PNP_PROFILE"):
        raise RuntimeError(
            "SFTWIN_PNP_PROFILE is not configured; set it before 2_after_play.py"
        )
    # This entrypoint may be executed repeatedly in one Isaac Script Editor
    # process. A run id is validation-attempt state, not bootstrap state, so
    # always issue a new one instead of replaying stale handoff files.
    run_id = f"pnp-{time.time_ns()}"
    environment["SFTWIN_PNP_RUN_ID"] = run_id
    runtime_dir = Path(
        environment.setdefault("SFTWIN_PNP_RUNTIME_DIR", "/tmp/sftwin_pnp_validation")
    )
    environment["SFTWIN_PNP_REGISTRATION_MANIFEST"] = str(
        runtime_dir / f"pnp_validation_{run_id}.json"
    )
    environment.setdefault(
        "SFTWIN_PNP_PLACE_CONFIRMATION",
        str(runtime_dir / "place_confirmation.json"),
    )
    environment.setdefault(
        "SFTWIN_PNP_DELETE_REQUEST",
        str(runtime_dir / "fixture_delete_request.json"),
    )
    environment.setdefault(
        "SFTWIN_PNP_DELETE_ACK",
        str(runtime_dir / "fixture_delete_ack.json"),
    )
    environment.setdefault(
        "SFTWIN_PNP_PLACE_EVENT_CONSUMED",
        str(runtime_dir / "place_result_event.consumed.json"),
    )
    environment.setdefault(
        "SFTWIN_PNP_PLACE_EVENT",
        str(runtime_dir / "place_result_event.json"),
    )
    environment.setdefault(
        "SFTWIN_PNP_MISSION_COMPLETE",
        str(runtime_dir / "mission_completion_event.json"),
    )
    environment.setdefault(
        "SFTWIN_PNP_PLANNING_SCENE_STATE",
        str(runtime_dir / "planning_scene_state.json"),
    )
    return {
        "SFTWIN_PNP_RUN_ID": run_id,
        "SFTWIN_PNP_RUNTIME_DIR": str(runtime_dir),
        "SFTWIN_PNP_REGISTRATION_MANIFEST": environment[
            "SFTWIN_PNP_REGISTRATION_MANIFEST"
        ],
        "SFTWIN_PNP_PLACE_CONFIRMATION": environment["SFTWIN_PNP_PLACE_CONFIRMATION"],
        "SFTWIN_PNP_DELETE_REQUEST": environment["SFTWIN_PNP_DELETE_REQUEST"],
        "SFTWIN_PNP_DELETE_ACK": environment["SFTWIN_PNP_DELETE_ACK"],
        "SFTWIN_PNP_PLACE_EVENT_CONSUMED": environment[
            "SFTWIN_PNP_PLACE_EVENT_CONSUMED"
        ],
        "SFTWIN_PNP_PLACE_EVENT": environment["SFTWIN_PNP_PLACE_EVENT"],
        "SFTWIN_PNP_MISSION_COMPLETE": environment["SFTWIN_PNP_MISSION_COMPLETE"],
        "SFTWIN_PNP_PLANNING_SCENE_STATE": environment[
            "SFTWIN_PNP_PLANNING_SCENE_STATE"
        ],
    }


def validate_bootstrap_environment(environ=None):
    """Fail before fixture creation when the common Isaac bootstrap was skipped."""
    environment = os.environ if environ is None else environ
    project_root = environment.get("SFTWIN_PROJECT_ROOT")
    if not project_root:
        raise RuntimeError("run 1_before_play.py before validation")
    domain = environment.get("ROS_DOMAIN_ID")
    if domain is None or not domain.isdigit() or not 0 <= int(domain) <= 232:
        raise RuntimeError("ROS_DOMAIN_ID is not configured by Isaac bootstrap")
    return {"SFTWIN_PROJECT_ROOT": project_root, "ROS_DOMAIN_ID": str(int(domain))}


def _complete_validation_start(
    runtime_session, profile, profile_path, environment, runtime
):
    """Create validation material only after the physical OPEN gate passes."""
    run = ValidationRun(
        profile=ValidationRunProfile.from_yaml(profile),
        runtime=runtime,
        run_id=environment["SFTWIN_PNP_RUN_ID"],
        clock_s=time.time,
        cleanup_confirmation_path=Path(environment["SFTWIN_PNP_PLACE_CONFIRMATION"]),
    )
    manifest = run.start()
    runtime_session.grasp_manager.pre_capture_stability = runtime.pre_capture_stability
    handoff_path = Path(environment["SFTWIN_PNP_REGISTRATION_MANIFEST"])
    handoff = RegistrationHandoff.from_manifest(manifest, str(profile_path))
    atomic_write_registration_handoff(handoff_path, handoff)
    globals()["pnp_validation_run"] = run
    globals()["pnp_validation_handoff"] = handoff
    globals()["pnp_validation_grasp_manager"] = runtime_session.grasp_manager
    cleanup_worker = CleanupRequestWorker(
        run,
        environment["SFTWIN_PNP_DELETE_REQUEST"],
        environment["SFTWIN_PNP_DELETE_ACK"],
    )
    globals()["pnp_validation_cleanup_worker"] = cleanup_worker
    mission_completion_worker = MissionCompletionEventWorker(
        run,
        environment["SFTWIN_PNP_MISSION_COMPLETE"],
        environment["SFTWIN_PNP_DELETE_ACK"],
    )
    globals()["pnp_validation_mission_completion_worker"] = mission_completion_worker
    place_event_worker = PlaceResultEventWorker(
        run.run_id,
        run.profile.target_id,
        environment["SFTWIN_PNP_PLACE_EVENT"],
        environment["SFTWIN_PNP_PLACE_EVENT_CONSUMED"],
        consume_place_result_event,
    )
    globals()["pnp_validation_place_event_worker"] = place_event_worker
    try:
        import omni.kit.app

        owner = globals().get("pnp_validation_session_owner")
        if owner is None:
            raise RuntimeError("Run 2_after_play.py before validation")
        globals()["pnp_validation_cleanup_subscription"] = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                lambda event: (
                    place_event_worker.poll(),
                    run.observe_post_release_position(),
                    cleanup_worker.poll(),
                    mission_completion_worker.poll(),
                )
            )
        )
        owner.register_subscription(
            "validation_cleanup", globals()["pnp_validation_cleanup_subscription"]
        )
    except Exception as error:
        raise RuntimeError(
            "failed to connect Isaac cleanup request worker to update loop"
        ) from error
    print("[PnP Diagnostic] FIXTURE_READY_FOR_REGISTRATION")
    print(f"[PnP Diagnostic] registration handoff: {handoff_path}")
    print(manifest.to_dict())
    return manifest


def _reuse_owned_fixture_on_rerun(runtime):
    """Keep an existing validation fixture's PhysX view valid across reruns.

    ``FixtureRuntime.spawn_cube`` historically deleted an owned stale prim
    before recreating it.  Deleting a prim that is still referenced by an
    Isaac physics tensor view invalidates that view and breaks the next
    controller/feedback tick.  This entrypoint-only policy adopts the owned
    fixture instead; foreign fixtures and active grasp joints remain fail
    closed.
    """
    original_spawn_cube = runtime.spawn_cube

    def spawn_or_adopt(self, prim_path, pose, dimensions):
        prim_path = str(prim_path)
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return original_spawn_cube(prim_path, pose, dimensions)
        if not self._is_owned(prim):
            canonical_path = "/World/SFTwin/ValidationFixture/Cube"
            if prim_path != canonical_path:
                raise RuntimeError("fixture prim path is already occupied")
            # The canonical validation path is the recovery boundary for a
            # fixture left by an earlier entrypoint run.  Re-establish the
            # validation marker instead of deleting a possibly live rigid
            # body and invalidating Isaac's physics tensor view.
            self._set_owner(prim)
        if self.stage.GetPrimAtPath(f"{prim_path}/sf_scripted_grasp_joint").IsValid():
            raise RuntimeError("cannot reuse fixture with an active grasp joint")
        self.registry.unregister(prim_path)
        self.adopt_existing_fixture(prim_path)
        self._authored_world_pose = pose
        self._transform_diagnostic_emitted = False
        if not self.pre_capture_stability.hold_for_registration():
            self._owned_path = None
            self.pre_capture_stability = None
            raise RuntimeError("failed to establish pre-capture fixture stability")

    runtime.spawn_cube = MethodType(spawn_or_adopt, runtime)


def run_current_stage():
    import omni.usd

    validate_bootstrap_environment()
    runtime_session = globals().get("runtime_session")
    if runtime_session is None:
        raise RuntimeError(
            "Run this diagnostic in the same Script Editor session as the active stage"
        )
    if runtime_session.grasp_config is None:
        raise RuntimeError(
            "Run 2_after_play.py with SFTWIN_PNP_PROFILE pointing at the validation profile"
        )
    if getattr(runtime_session.gripper, "feedback_generation", None) is None:
        raise ValidationStartRejected(
            "stale gripper runtime session: reload the current 2_after_play.py "
            "before running validation"
        )
    owner = globals().get("pnp_validation_session_owner")
    if owner is None:
        raise RuntimeError("Run 2_after_play.py before validation")

    project_root = Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()
    profile_path = Path(os.environ["SFTWIN_PNP_PROFILE"])
    profile = load_validation_profile(profile_path)
    validation_environment = configure_validation_environment()
    planning_path = (
        project_root
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/isaac_planning.yaml"
    )
    planning = yaml.safe_load(planning_path.read_text())

    runtime = IsaacFixtureRuntime(
        omni.usd.get_context().get_stage(),
        world_to_base=(
            planning["world_to_base"]["x"],
            planning["world_to_base"]["y"],
            planning["world_to_base"]["z"],
            planning["world_to_base"]["yaw"],
        ),
        registry=runtime_session.registry,
    )
    from graph_builder.graph_contract import GRAPH_PATH
    from graph_builder.graph_lifecycle import release_graph_controller_node
    from graph_builder.ros_action_graph import build_ros_action_graph

    print(
        "[PnP Diagnostic] graph recovery API: "
        f"{build_ros_action_graph.__code__.co_filename} "
        f"{inspect.signature(build_ros_action_graph)}"
    )
    from pnp_validation.graph_view_recovery import delete_with_graph_rebuild

    def rebuild_action_graph():
        import omni.graph.core as og
        import usdrt.Sdf
        from pxr import UsdGeom, UsdPhysics

        build_ros_action_graph(
            stage=runtime.stage,
            controller=og.Controller,
            sdf_path=usdrt.Sdf.Path,
            usd_geom=UsdGeom,
            usd_physics=UsdPhysics,
            # Fixture removal changes the physics scene and invalidates all
            # existing articulation views. Keep the non-physics ROS publishers
            # alive after terminal cleanup; the normal before_play ActionGraph build step restores
            # the complete command graph before another motion run.
            enable_arm_command_subscriber=False,
        )

    def delete_fixture_and_recover_graph(prim_path):
        def release_articulation_controller_view():
            import omni.graph.core as og

            release_graph_controller_node(og.GraphController, GRAPH_PATH)

        return delete_with_graph_rebuild(
            stage=runtime.stage,
            graph_path=GRAPH_PATH,
            release_graph_view=release_articulation_controller_view,
            delete_fixture=lambda: runtime._default_delete_prim(prim_path),
            rebuild_graph=rebuild_action_graph,
        )

    # Release the OmniGraph node instance so Isaac invokes its PhysX view
    # cleanup before a terminal fixture mutation invalidates the scene.
    runtime._delete_prim = delete_fixture_and_recover_graph
    owner.register_validation(runtime, validation_environment)
    _reuse_owned_fixture_on_rerun(runtime)
    baseline_generation = request_validation_start_state(runtime_session.grasp_manager)
    try:
        import omni.kit.app

        previous_subscription = globals().get("pnp_validation_start_subscription")
        if previous_subscription is not None:
            previous_subscription.unsubscribe()
        started_at = time.monotonic()
        deadline = started_at + float(
            profile["validation"].get("open_readiness_timeout_s", 2.0)
        )

        def wait_for_physical_open(event):
            try:
                validation_start_state_ready(
                    runtime_session.grasp_manager, baseline_generation
                )
            except ValidationStartPending as error:
                if time.monotonic() < deadline:
                    return
                subscription.unsubscribe()
                globals()["pnp_validation_start_subscription"] = None
                diagnostic = validation_start_diagnostic(
                    runtime_session.grasp_manager,
                    baseline_generation,
                    str(error),
                )
                diagnostic["elapsed_wall_s"] = time.monotonic() - started_at
                print(f"[PnP Diagnostic] physical OPEN readiness timeout: {diagnostic}")
                raise RuntimeError(
                    "validation start physical OPEN readiness timeout"
                ) from error
            except ValidationStartRejected as error:
                subscription.unsubscribe()
                globals()["pnp_validation_start_subscription"] = None
                diagnostic = validation_start_diagnostic(
                    runtime_session.grasp_manager,
                    baseline_generation,
                    str(error),
                )
                diagnostic["elapsed_wall_s"] = time.monotonic() - started_at
                print(
                    f"[PnP Diagnostic] physical OPEN readiness rejected: {diagnostic}"
                )
                raise
            subscription.unsubscribe()
            globals()["pnp_validation_start_subscription"] = None
            _complete_validation_start(
                runtime_session,
                profile,
                profile_path,
                validation_environment,
                runtime,
            )

        subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(wait_for_physical_open)
        )
        globals()["pnp_validation_start_subscription"] = subscription
        owner.register_subscription("validation_start", subscription)
    except Exception as error:
        raise RuntimeError(
            "failed to connect validation physical OPEN gate to update loop"
        ) from error
    print(
        "[PnP Diagnostic] waiting for fresh physical OPEN feedback before fixture spawn"
    )
    return None


def record_place_result(
    *,
    success: bool,
    actual_pose=None,
):
    """Publish the live PLACE decision through the existing ValidationRun policy."""
    run = globals().get("pnp_validation_run")
    if run is None:
        raise RuntimeError("run_current_stage must create the validation run first")
    from pnp_validation.fixture import Pose

    grasp_manager = globals().get("pnp_validation_grasp_manager")
    if grasp_manager is None:
        raise RuntimeError("validation gripper runtime is not connected")
    observation = grasp_manager.holding_observation()
    fresh_released = observation.fresh() and observation.state.value == "released"
    detach_confirmed = grasp_manager.attach_joint_path is None
    ownership_unambiguous = run.ownership.owns(run.prim_path)
    expected_base = Pose(*run.profile.expected_fixture_release_position_base)
    expected_world = run.runtime.transform.base_to_world(expected_base)
    decision = run.on_place_result(
        success=success,
        fresh_released=fresh_released,
        detach_confirmed=detach_confirmed,
        ownership_unambiguous=ownership_unambiguous,
        actual_pose=actual_pose,
        expected_position_world=(expected_world.x, expected_world.y, expected_world.z),
    )
    return decision


def consume_place_result_event(event):
    """Bridge one successful ROS PLACE fact into the canonical Isaac policy."""
    if not (
        event.get("status") == "SUCCESS"
        and event.get("release_motion_completed") is True
        and event.get("fresh_released") is True
        and event.get("detach_confirmed") is True
    ):
        return False
    return record_place_result(success=True).eligible


class ValidationRunProfile:
    """Convert the YAML profile into the pure fixture profile model."""

    @staticmethod
    def from_yaml(config):
        return fixture_profile_from_validation_config(config)


if __name__ == "__main__":
    run_current_stage()
