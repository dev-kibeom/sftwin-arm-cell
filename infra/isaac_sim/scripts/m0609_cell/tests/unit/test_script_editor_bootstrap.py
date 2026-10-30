import importlib
import runpy
import shutil
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
import yaml
from shared import script_editor_bootstrap as bootstrap

PROJECT_ROOT = Path(__file__).resolve().parents[6]
CELL_ROOT = PROJECT_ROOT / "infra/isaac_sim/scripts/m0609_cell"


def test_explicit_project_root_resolves_cell_module_root():
    assert bootstrap.module_root_from_project_root(PROJECT_ROOT) == CELL_ROOT


def test_explicit_invalid_project_root_is_rejected():
    with pytest.raises(RuntimeError, match="not a directory"):
        bootstrap.module_root_from_project_root("/missing/sftwin")


def test_script_editor_temp_path_requires_explicit_project_root():
    with pytest.raises(RuntimeError, match="SFTWIN_PROJECT_ROOT"):
        bootstrap.discover_module_root("/tmp/carb.o6orAJ/script_1789079845.py", {})


def test_script_editor_temp_path_uses_explicit_project_root():
    assert (
        bootstrap.discover_module_root(
            "/tmp/carb.o6orAJ/script_1789079845.py",
            {"SFTWIN_PROJECT_ROOT": str(PROJECT_ROOT)},
        )
        == CELL_ROOT
    )


def test_source_path_resolution_does_not_depend_on_current_directory():
    assert (
        bootstrap.discover_module_root(
            CELL_ROOT / "camera_tooling/entrypoints/inspect_camera.py", {}
        )
        == CELL_ROOT
    )


def test_after_play_does_not_change_before_play_diagnostics(tmp_path, monkeypatch):
    descriptor = tmp_path / "session.json"
    descriptor.write_text(
        '{"status":"open","diagnostic_selection":["motion"]}',
        encoding="utf-8",
    )
    monkeypatch.setenv("SFTWIN_LIVE_TEST_SESSION_PATH", str(descriptor))
    monkeypatch.setenv("SFTWIN_DIAGNOSTICS", "none")
    entrypoint = ModuleType("shared.role_entrypoint")
    observed = {}

    def run(role, namespace):
        observed["role"] = role
        observed["selection"] = __import__("os").environ["SFTWIN_DIAGNOSTICS"]

    entrypoint.run = run
    monkeypatch.setitem(sys.modules, "shared.role_entrypoint", entrypoint)
    import shared

    monkeypatch.setattr(shared, "role_entrypoint", entrypoint, raising=False)
    monkeypatch.setattr("importlib.reload", lambda module: module)
    runpy.run_path(str(CELL_ROOT / "2_after_play.py"))
    assert observed == {"role": "after_play", "selection": "none"}


def test_headless_after_play_uses_physics_only_role(monkeypatch):
    entrypoint = ModuleType("shared.role_entrypoint")
    observed = {}
    entrypoint.run = lambda role, _namespace: observed.setdefault("role", role)
    monkeypatch.setitem(sys.modules, "shared.role_entrypoint", entrypoint)
    import shared

    monkeypatch.setattr(shared, "role_entrypoint", entrypoint, raising=False)
    monkeypatch.setattr("importlib.reload", lambda module: module)
    runpy.run_path(str(CELL_ROOT / "2_after_play_headless.py"))
    assert observed == {"role": "after_play_headless"}


def test_package_import_cache_can_be_cleared_without_removing_prefix_siblings(
    monkeypatch,
):
    package = "_test_graph_builder"
    names = (package, f"{package}.contract", f"{package}_other")
    for name in names:
        monkeypatch.setitem(sys.modules, name, ModuleType(name))

    bootstrap.clear_imported_package(package)

    assert package not in sys.modules
    assert f"{package}.contract" not in sys.modules
    assert f"{package}_other" in sys.modules


def test_package_bytecode_cache_is_removed_for_script_editor_reload(tmp_path):
    package = tmp_path / "gripper_runtime"
    cache = package / "__pycache__"
    cache.mkdir(parents=True)
    bytecode = cache / "session.cpython-310.pyc"
    bytecode.write_bytes(b"stale bytecode")

    bootstrap.clear_package_bytecode(package)

    assert not bytecode.exists()


def test_gripper_bootstrap_replaces_session_and_reloads_split_packages(
    tmp_path, monkeypatch
):
    """Run the real entrypoint with only Isaac's Kit boundary replaced."""
    project_root = tmp_path / "checkout"
    cell_root = project_root / "infra/isaac_sim/scripts/m0609_cell"
    cell_root.mkdir(parents=True)
    for package in (
        "shared",
        "gripper_runtime",
        "pnp_validation",
        "graph_builder",
        "runtime_steps",
    ):
        shutil.copytree(CELL_ROOT / package, cell_root / package)
    config_root = project_root / "ros2_ws/src/arm_cell/arm_cell_bringup/config"
    config_root.mkdir(parents=True)
    shutil.copy2(
        PROJECT_ROOT
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml",
        config_root / "pnp_validation_profile.yaml",
    )
    profile = config_root / "pnp_validation_profile.yaml"
    live_session_descriptor = tmp_path / "diagnostic_records" / "session.json"
    profile_data = yaml.safe_load(profile.read_text(encoding="utf-8"))
    recipe_directory = config_root / profile_data["orchestration"]["recipe_directory"]
    source_recipe_directory = (
        PROJECT_ROOT
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config"
        / profile_data["orchestration"]["recipe_directory"]
    )
    shutil.copytree(source_recipe_directory, recipe_directory)

    class Subscription:
        def __init__(self, callback):
            self.callback = callback
            self.unsubscribe_calls = 0

        def unsubscribe(self):
            self.unsubscribe_calls += 1

    class EventStream:
        def __init__(self):
            self.subscriptions = []

        def create_subscription_to_pop(self, callback):
            subscription = Subscription(callback)
            self.subscriptions.append(subscription)
            return subscription

    event_stream = EventStream()
    physx = SimpleNamespace(
        get_simulation_event_stream_v2=lambda: event_stream,
    )
    omni = ModuleType("omni")
    omni.__path__ = []
    omni_physx = ModuleType("omni.physx")
    omni_physx.get_physx_interface = lambda: physx
    bindings = ModuleType("omni.physx.bindings")
    bindings.__path__ = []
    physx_bindings = ModuleType("omni.physx.bindings._physx")
    physx_bindings.SimulationEvent = SimpleNamespace(STOPPED=4)
    omni.physx = omni_physx
    omni_physx.bindings = bindings
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.physx", omni_physx)
    monkeypatch.setitem(sys.modules, "omni.physx.bindings", bindings)
    monkeypatch.setitem(sys.modules, "omni.physx.bindings._physx", physx_bindings)

    old_modules = {
        name: module
        for name, module in tuple(sys.modules.items())
        if name.split(".", 1)[0]
        in {"gripper_runtime", "pnp_validation", "graph_builder"}
    }
    old_owner_module = sys.modules.get("shared.script_editor_bootstrap")
    for name in old_modules:
        sys.modules.pop(name, None)

    class PreviousSession:
        _status_sequence = 8
        grasp_manager = SimpleNamespace(
            holding_observation=lambda: SimpleNamespace(sequence=21)
        )

        def __init__(self):
            self.shutdown_calls = 0

        def shutdown(self):
            self.shutdown_calls += 1

    previous_session = PreviousSession()
    created = []

    class NewSession:
        def __init__(self, initial_sequence):
            self._status_sequence = initial_sequence
            self.gripper = SimpleNamespace(feedback_generation=0, open_calls=0)
            self.gripper.open = lambda: setattr(
                self.gripper, "open_calls", self.gripper.open_calls + 1
            )
            self.grasp_manager = SimpleNamespace(
                candidate_source=SimpleNamespace(require_motion_reference=False)
            )
            self.grasp_config = object()
            self.sim_adapter = object()
            self.physics_sub = Subscription(self.on_physics_step)
            self.unsubscribe_calls = 0
            self.subscribe_calls = 1

        def on_physics_step(self, _dt):
            self.gripper.feedback_generation += 1

        def _unsubscribe(self):
            self.unsubscribe_calls += 1

        def subscribe(self):
            self.subscribe_calls += 1
            self.physics_sub = Subscription(self.on_physics_step)

    original_reload = importlib.reload

    def reload_and_install_session_factory(module):
        reloaded = original_reload(module)
        if module.__name__ == "gripper_runtime.session":

            def create_session(_physx, **kwargs):
                session = NewSession(kwargs["initial_status_sequence"])
                created.append((session, kwargs))
                return session

            reloaded.create_gripper_runtime_session = create_session
        return reloaded

    monkeypatch.setattr(importlib, "reload", reload_and_install_session_factory)
    monkeypatch.setenv("SFTWIN_PROJECT_ROOT", str(project_root))
    monkeypatch.setenv("SFTWIN_RUNTIME_MODE", "diagnostic")
    monkeypatch.setenv("SFTWIN_PNP_PROFILE", str(profile))
    monkeypatch.setenv("SFTWIN_LIVE_TEST_SESSION_PATH", str(live_session_descriptor))
    namespace = {"runtime_session": previous_session}
    entrypoint = cell_root / "runtime_steps/bootstrap_gripper.py"
    original_path = list(sys.path)
    try:
        namespace = runpy.run_path(str(entrypoint), init_globals=namespace)

        assert previous_session.shutdown_calls == 1
        assert len(created) == 1
        session, kwargs = created[0]
        assert kwargs["initial_status_sequence"] >= 21
        assert kwargs["diagnostic_output_directory"] == (
            live_session_descriptor.parent / "gripper"
        )
        assert session.unsubscribe_calls == 1
        assert session.subscribe_calls == 2
        assert session.gripper.open_calls == 1
        assert session.grasp_manager.candidate_source.require_motion_reference is True
        assert len(event_stream.subscriptions) == 1
        for module_name in (
            "gripper_runtime.grasp_stage",
            "gripper_runtime.grasp_joint",
            "pnp_validation.profile",
            "graph_builder.graph_contract",
        ):
            module = sys.modules[module_name]
            assert Path(module.__file__).is_relative_to(cell_root)
        assert namespace["runtime_session"] is session
        assert namespace["pnp_validation_session_owner"]._session is session
    finally:
        for name in tuple(sys.modules):
            if name.split(".", 1)[0] in {
                "gripper_runtime",
                "pnp_validation",
                "graph_builder",
            }:
                sys.modules.pop(name, None)
        sys.modules.update(old_modules)
        if old_owner_module is None:
            sys.modules.pop("shared.script_editor_bootstrap", None)
        else:
            sys.modules["shared.script_editor_bootstrap"] = old_owner_module
        sys.path[:] = original_path
