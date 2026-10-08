"""Attach the validation-private fixture command worker to a live Isaac stage.

This Script Editor entrypoint intentionally does not build, reset, or stop the
stage.  It reuses the existing runtime_session created by 2_after_play.py.
"""

import os
import importlib
from pathlib import Path

import yaml


def attach():
    import omni.kit.app
    import omni.usd
    import pnp_validation.live_fixture as live_fixture
    import pnp_validation.run_validation as run_validation

    importlib.reload(live_fixture)
    importlib.reload(run_validation)

    session = globals().get("runtime_session")
    if session is None:
        raise RuntimeError("2_after_play.py runtime_session is required")
    profile_path = Path(os.environ["SFTWIN_PNP_PROFILE"])
    profile = run_validation.load_validation_profile(profile_path)
    project_root = Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()
    planning_path = (
        project_root
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/isaac_planning.yaml"
    )
    planning = yaml.safe_load(planning_path.read_text())
    runtime = run_validation.IsaacFixtureRuntime(
        omni.usd.get_context().get_stage(),
        world_to_base=(
            planning["world_to_base"]["x"],
            planning["world_to_base"]["y"],
            planning["world_to_base"]["z"],
            planning["world_to_base"]["yaw"],
        ),
        registry=session.registry,
    )
    fixture_path = "/World/SFTwin/ValidationFixture/Cube"
    if runtime.stage.GetPrimAtPath(fixture_path).IsValid():
        runtime.adopt_existing_fixture(fixture_path)
    validation_profile = run_validation.fixture_profile_from_validation_config(profile)
    runtime_dir = Path(
        os.environ.get("SFTWIN_PNP_RUNTIME_DIR", "/tmp/sftwin_pnp_validation")
    )
    worker = run_validation.IsaacFixtureCommandWorker(
        runtime,
        command_path=runtime_dir / "fixture_command.json",
        response_path=runtime_dir / "fixture_response.json",
        run_id=os.environ.get("SFTWIN_PNP_RUN_ID", "live-fixture-control"),
        fixture_id=fixture_path,
        fixture_dimensions=validation_profile.fixture_dimensions,
        seed=validation_profile.seed,
        clearance_m=validation_profile.spawn_clearance_m,
    )
    old = globals().get("pnp_fixture_command_subscription")
    if old is not None:
        old.unsubscribe()
    subscription = (
        omni.kit.app.get_app()
        .get_update_event_stream()
        .create_subscription_to_pop(lambda event: worker.poll())
    )
    globals()["pnp_fixture_runtime"] = runtime
    globals()["pnp_fixture_command_worker"] = worker
    globals()["pnp_fixture_command_subscription"] = subscription
    print("[PnP Diagnostic] validation fixture command runtime attached")
    print(f"[PnP Diagnostic] command path: {runtime_dir / 'fixture_command.json'}")
    print(f"[PnP Diagnostic] response path: {runtime_dir / 'fixture_response.json'}")
    return worker


if __name__ == "__main__":
    attach()
