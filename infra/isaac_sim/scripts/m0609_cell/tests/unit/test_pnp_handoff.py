import importlib.util
import json
from pathlib import Path

import pytest

from pnp_validation.fixture import Pose, PoseReference, ValidationManifest
from pnp_validation.handoff import (
    HANDOFF_SCHEMA_VERSION,
    RegistrationHandoff,
    atomic_write_json,
    atomic_write_registration_handoff,
)
from pnp_validation.editor_lifecycle import ValidationSessionOwner


class _LifecycleStream:
    def __init__(self):
        self.subscriptions = []

    def create_subscription_to_pop(self, callback):
        del callback
        subscription = type(
            "Subscription",
            (),
            {
                "unsubscribe_calls": 0,
                "unsubscribe": lambda self: setattr(
                    self, "unsubscribe_calls", self.unsubscribe_calls + 1
                ),
            },
        )()
        self.subscriptions.append(subscription)
        return subscription


def _manifest():
    return ValidationManifest(
        run_id="run-1",
        seed=17,
        target_id="cube_profile",
        fixture_prim_identity="/World/SFTwin/ValidationFixture/Cube",
        dimensions=(0.07, 0.07, 0.08),
        spawn_pose=Pose(0.2, -0.1, 0.8, 0.25),
        registered_top_surface_pose=PoseReference(0.2, -0.1, 0.84, 0.25),
        has_target_yaw=True,
        ttl_s=30.0,
        capture_profile_reference="profile.yaml:capture",
        tolerance_profile_reference="profile.yaml:tolerances",
    )


def test_registration_handoff_has_versioned_manifest_schema():
    handoff = RegistrationHandoff.from_manifest(_manifest(), "profile.yaml")

    data = handoff.to_dict()

    assert data["schema_version"] == HANDOFF_SCHEMA_VERSION
    assert data["status"] == "FIXTURE_READY_FOR_REGISTRATION"
    assert data["request"]["pose"]["frame_id"] == "base_link"
    assert data["request"]["run_id"] == "run-1"
    assert data["request"]["target_id"] == "cube_profile"
    assert data["request"]["ttl_s"] == 30.0


def test_registration_handoff_is_written_atomically(tmp_path):
    path = tmp_path / "registration_request.json"

    atomic_write_registration_handoff(
        path, RegistrationHandoff.from_manifest(_manifest(), "profile.yaml")
    )

    assert json.loads(path.read_text())["run_id"] == "run-1"
    assert not list(tmp_path.glob("*.tmp"))


def test_cleanup_json_is_written_atomically(tmp_path):
    path = tmp_path / "cleanup.json"

    atomic_write_json(path, {"status": "AUTHORIZED", "decision_id": "d-1"})

    assert json.loads(path.read_text()) == {
        "status": "AUTHORIZED",
        "decision_id": "d-1",
    }
    assert not list(tmp_path.glob("*.tmp"))


def test_isaac_entrypoint_has_no_direct_ros_python_dependency():
    import ast

    source = (
        Path(__file__).parents[2] / "pnp_validation/entrypoints/start_validation.py"
    )
    tree = ast.parse(source.read_text(encoding="utf-8"))
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add(node.module)

    forbidden_roots = {"rclpy", "arm_cell_vision"}
    assert (
        not {module.split(".", 1)[0] for module in imported_modules} & forbidden_roots
    )


def test_validation_runner_owns_validation_specific_environment_defaults():
    source = (
        Path(__file__).parents[2] / "pnp_validation/entrypoints/start_validation.py"
    )
    spec = importlib.util.spec_from_file_location("run_pnp_validation", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    environment = {"SFTWIN_PNP_PROFILE": "/tmp/project/profile.yaml"}

    result = module.configure_validation_environment(environ=environment)

    assert "SFTWIN_PNP_PROFILE" not in result
    assert environment["SFTWIN_PNP_PROFILE"] == "/tmp/project/profile.yaml"
    assert result["SFTWIN_PNP_RUN_ID"].startswith("pnp-")
    assert result["SFTWIN_PNP_RUNTIME_DIR"] == "/tmp/sftwin_pnp_validation"
    assert result["SFTWIN_PNP_REGISTRATION_MANIFEST"].endswith(
        f"{result['SFTWIN_PNP_RUN_ID']}.json"
    )
    assert result["SFTWIN_PNP_PLANNING_SCENE_STATE"].endswith(
        "planning_scene_state.json"
    )
    assert result["SFTWIN_PNP_MISSION_COMPLETE"].endswith(
        "mission_completion_event.json"
    )


def test_validation_runner_allocates_a_fresh_run_on_script_editor_rerun():
    source = (
        Path(__file__).parents[2] / "pnp_validation/entrypoints/start_validation.py"
    )
    spec = importlib.util.spec_from_file_location("run_pnp_validation_rerun", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    environment = {
        "SFTWIN_PNP_PROFILE": "/tmp/project/profile.yaml",
        "SFTWIN_PNP_RUN_ID": "stale-run",
        "SFTWIN_PNP_REGISTRATION_MANIFEST": "/tmp/stale-handoff.json",
    }

    first = module.configure_validation_environment(environ=environment)
    second = module.configure_validation_environment(environ=environment)

    assert first["SFTWIN_PNP_RUN_ID"] != second["SFTWIN_PNP_RUN_ID"]
    assert environment["SFTWIN_PNP_RUN_ID"] == second["SFTWIN_PNP_RUN_ID"]
    assert second["SFTWIN_PNP_REGISTRATION_MANIFEST"].endswith(
        f"{second['SFTWIN_PNP_RUN_ID']}.json"
    )


def test_stop_owner_cleans_owned_state_and_artifacts(tmp_path):
    source = (
        Path(__file__).parents[2] / "pnp_validation/entrypoints/start_validation.py"
    )
    spec = importlib.util.spec_from_file_location("run_pnp_validation_teardown", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class Prim:
        def __init__(self, valid, owned):
            self.valid = valid
            self.owned = owned

        def IsValid(self):
            return self.valid

    class Stage:
        def GetPrimAtPath(self, path):
            return Prim(valid=True, owned=True)

    class Runtime:
        def __init__(self):
            self.stage = Stage()
            self._owned_path = "/World/SFTwin/ValidationFixture/Cube"
            self.deleted = []

        def delete_prim(self, path):
            self.deleted.append(path)
            self._owned_path = None
            return True

        def _is_owned(self, prim):
            return prim.owned

    simulation_events = _LifecycleStream()
    validation_subscription = type(
        "Subscription", (), {"unsubscribe": lambda self: setattr(self, "closed", True)}
    )()

    environment = module.configure_validation_environment(
        {
            "SFTWIN_PNP_PROFILE": "/tmp/project/profile.yaml",
            "SFTWIN_PNP_RUNTIME_DIR": str(tmp_path),
        }
    )
    runtime = Runtime()
    session = type(
        "Session",
        (),
        {
            "registry": type(
                "Registry",
                (),
                {"paths": lambda self: (), "unregister": lambda self, path: None},
            )(),
            "grasp_manager": type("Manager", (), {"attach_joint_path": None})(),
            "physics_sub": None,
            "shutdown": lambda self: True,
        },
    )()
    namespace = {"pnp_validation_run": object()}
    owner = ValidationSessionOwner(simulation_events, stopped_event_type="STOP")
    owner.bind_namespace(namespace).register_session(session)
    owner.start()
    owner.register_subscription("validation", validation_subscription)
    owner.register_validation(runtime, environment)
    for path in environment.values():
        if path.endswith(".json"):
            Path(path).write_text("validation-owned")

    assert owner.teardown() is True
    assert runtime.deleted == ["/World/SFTwin/ValidationFixture/Cube"]
    assert namespace["pnp_validation_run"] is None
    assert validation_subscription.closed is True
    assert simulation_events.subscriptions[0].unsubscribe_calls == 0
    assert not list(tmp_path.glob("*.json"))


def test_stop_owner_preserves_foreign_fixture(tmp_path):
    class Prim:
        def IsValid(self):
            return True

    class Runtime:
        stage = type("Stage", (), {"GetPrimAtPath": lambda self, path: Prim()})()
        _owned_path = None

        def _is_owned(self, prim):
            return False

        def delete_prim(self, path):
            raise AssertionError(f"foreign fixture deleted: {path}")

    environment = {
        key: str(tmp_path / f"{key}.json")
        for key in ValidationSessionOwner.ARTIFACT_KEYS
    }
    runtime = Runtime()
    session = type(
        "Session",
        (),
        {
            "registry": None,
            "grasp_manager": type("Manager", (), {"attach_joint_path": None})(),
            "shutdown": lambda self: True,
        },
    )()
    owner = ValidationSessionOwner(object(), stopped_event_type="STOP")
    owner.register_session(session)
    owner.register_validation(runtime, environment)

    assert owner.teardown() is True


def test_stop_owner_rechecks_stale_owned_path_before_deleting_foreign_prim(tmp_path):
    class ForeignPrim:
        def IsValid(self):
            return True

    class Runtime:
        _owned_path = "/World/SFTwin/ValidationFixture/Cube"
        stage = type("Stage", (), {"GetPrimAtPath": lambda self, path: ForeignPrim()})()

        def _is_owned(self, prim):
            return False

        def delete_prim(self, path):
            raise AssertionError(f"foreign fixture deleted: {path}")

    environment = {
        key: str(tmp_path / f"{key}.json")
        for key in ValidationSessionOwner.ARTIFACT_KEYS
    }
    session = type(
        "Session",
        (),
        {
            "registry": None,
            "grasp_manager": type("Manager", (), {"attach_joint_path": None})(),
            "shutdown": lambda self: True,
        },
    )()
    owner = ValidationSessionOwner(object(), stopped_event_type="STOP")
    owner.register_session(session)
    owner.register_validation(Runtime(), environment)

    assert owner.teardown() is False


def test_validation_runner_requires_bootstrap_owned_profile():
    source = (
        Path(__file__).parents[2] / "pnp_validation/entrypoints/start_validation.py"
    )
    spec = importlib.util.spec_from_file_location("run_pnp_validation", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    with pytest.raises(RuntimeError, match="1_before_play"):
        module.configure_validation_environment(environ={})


def test_validation_runner_rejects_missing_common_bootstrap_environment():
    source = (
        Path(__file__).parents[2] / "pnp_validation/entrypoints/start_validation.py"
    )
    spec = importlib.util.spec_from_file_location("run_pnp_validation", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    with pytest.raises(RuntimeError, match="1_before_play"):
        module.validate_bootstrap_environment({})


def test_validation_runner_reports_effective_bootstrap_domain():
    source = (
        Path(__file__).parents[2] / "pnp_validation/entrypoints/start_validation.py"
    )
    spec = importlib.util.spec_from_file_location("run_pnp_validation", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    result = module.validate_bootstrap_environment(
        {"SFTWIN_PROJECT_ROOT": "/tmp/project", "ROS_DOMAIN_ID": "0"}
    )

    assert result["ROS_DOMAIN_ID"] == "0"
