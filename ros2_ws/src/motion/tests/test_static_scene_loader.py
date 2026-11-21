"""Focused contract tests for static-scene loader mutation and readback."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
import pytest


def load_loader_module():
    source_package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "load_static_scene", source_package / "scripts/load_static_scene.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def artifact():
    return {
        "schema_version": 1,
        "frame_id": "world",
        "objects": [
            {
                "id": "selected_box",
                "geometry": {"type": "box", "dimensions_m": [0.2, 0.3, 0.4]},
                "pose": {
                    "position_m": [1, 2, 3],
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
            }
        ],
    }


def unrelated_object():
    object_ = CollisionObject()
    object_.id = "unrelated_object"
    object_.header.frame_id = "world"
    return object_


class FakePlanningSceneServices:
    def __init__(self, omit_selected_on_readback=False, nominal_valid=True):
        self.objects = {"unrelated_object": unrelated_object()}
        self.omit_selected_on_readback = omit_selected_on_readback
        self.nominal_valid = nominal_valid

    def apply(self, request):
        assert request.scene.is_diff
        for object_ in request.scene.world.collision_objects:
            self.objects[object_.id] = object_
        return SimpleNamespace(success=True)

    def read(self, _request):
        objects = list(self.objects.values())
        if self.omit_selected_on_readback:
            objects = [object_ for object_ in objects if object_.id != "selected_box"]
        scene = PlanningScene()
        scene.world.collision_objects = objects
        return SimpleNamespace(scene=scene)

    def validity(self, _request):
        return SimpleNamespace(valid=self.nominal_valid)


class IdentityTransformBuffer:
    @staticmethod
    def lookup_transform(target_frame, source_frame, _time, **_kwargs):
        assert (target_frame, source_frame) == ("world", "world")
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        return transform


def loader_with_services(services):
    loader = SimpleNamespace(
        artifact=artifact(),
        planning_frame="world",
        apply_client="apply",
        read_client="read",
        validity_client="validity",
        tf_buffer=IdentityTransformBuffer(),
    )

    def call(client, request):
        if client == "apply":
            return services.apply(request)
        if client == "read":
            return services.read(request)
        return services.validity(request)

    loader.lookup_transform_when_available = (
        lambda target, source: loader.tf_buffer.lookup_transform(target, source, None)
    )
    loader.call = call
    return loader


def test_loader_reapplication_preserves_unrelated_planning_scene_state():
    module = load_loader_module()
    services = FakePlanningSceneServices()
    loader = loader_with_services(services)

    module.StaticSceneLoader.apply_and_verify(loader)
    module.StaticSceneLoader.apply_and_verify(loader)

    assert set(services.objects) == {"selected_box", "unrelated_object"}


def test_loader_fails_explicitly_when_selected_object_is_missing_from_readback():
    module = load_loader_module()
    services = FakePlanningSceneServices(omit_selected_on_readback=True)
    loader = loader_with_services(services)

    with pytest.raises(module.StaticSceneLoadError, match="missing after apply"):
        module.StaticSceneLoader.apply_and_verify(loader)


def test_loader_checks_nominal_robot_state_validity_when_enabled():
    module = load_loader_module()
    services = FakePlanningSceneServices(nominal_valid=False)
    loader = loader_with_services(services)
    loader.verify_nominal_state = True

    with pytest.raises(
        module.StaticSceneLoadError, match="nominal current robot state"
    ):
        module.StaticSceneLoader.apply_and_verify(loader)


class ModelFrameOnlyServices(FakePlanningSceneServices):
    """MoveIt can apply model-frame geometry before its TF monitor is ready."""

    def apply(self, request):
        if any(obj.header.frame_id != "base_link"
               for obj in request.scene.world.collision_objects):
            return SimpleNamespace(success=False)
        return super().apply(request)


def test_loader_applies_world_geometry_in_model_frame_before_moveit_tf_is_ready():
    module = load_loader_module()
    services = ModelFrameOnlyServices()
    loader = loader_with_services(services)
    loader.planning_frame = "base_link"
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "world"
    transform.transform.translation.x = 10.0
    transform.transform.translation.y = -4.0
    transform.transform.translation.z = 0.5
    transform.transform.rotation.z = 2 ** -0.5
    transform.transform.rotation.w = 2 ** -0.5

    def lookup(target, source):
        assert (target, source) == ("base_link", "world")
        return transform

    loader.lookup_transform_when_available = lookup
    module.StaticSceneLoader.apply_and_verify(loader)

    obj = services.objects["selected_box"]
    assert obj.header.frame_id == "base_link"
    assert [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z] == (
        pytest.approx([8.0, -3.0, 3.5])
    )
    assert [obj.pose.orientation.z, obj.pose.orientation.w] == (
        pytest.approx([2 ** -0.5, 2 ** -0.5])
    )
    assert list(obj.primitives[0].dimensions) == [0.2, 0.3, 0.4]
    assert obj.primitive_poses[0].orientation.w == 1.0
    assert loader.artifact["objects"][0]["pose"]["position_m"] == [1.0, 2.0, 3.0]
    assert "unrelated_object" in services.objects


def test_loader_does_not_mutate_scene_when_source_transform_is_unavailable():
    module = load_loader_module()
    services = FakePlanningSceneServices()
    loader = loader_with_services(services)
    loader.planning_frame = "base_link"

    def unavailable(_target, _source):
        raise module.StaticSceneLoadError("TF unavailable before deadline")

    loader.lookup_transform_when_available = unavailable
    with pytest.raises(module.StaticSceneLoadError, match="TF unavailable"):
        module.StaticSceneLoader.apply_and_verify(loader)
    assert set(services.objects) == {"unrelated_object"}


def test_loader_keeps_moveit_scene_rejection_fatal_after_frame_resolution():
    module = load_loader_module()
    services = ModelFrameOnlyServices()
    loader = loader_with_services(services)

    with pytest.raises(module.StaticSceneLoadError, match="MoveIt rejected"):
        module.StaticSceneLoader.apply_and_verify(loader)
    assert set(services.objects) == {"unrelated_object"}


def test_transform_wait_processes_delayed_tf_before_returning(monkeypatch):
    module = load_loader_module()
    from tf2_ros import Buffer

    loader = SimpleNamespace(tf_buffer=Buffer(), tf_wait_timeout_s=1.0)
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "world"
    transform.transform.rotation.w = 1.0
    transform.transform.translation.x = 4.0
    updates = []

    def spin_once(_node, timeout_sec):
        updates.append(timeout_sec)
        if len(updates) == 3:
            loader.tf_buffer.set_transform_static(transform, "test")

    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "spin_once", spin_once)
    received = module.StaticSceneLoader.lookup_transform_when_available(
        loader, "base_link", "world"
    )
    assert received.transform.translation.x == 4.0
    assert len(updates) == 3


def test_transform_wait_fails_within_deadline_when_tf_never_arrives(monkeypatch):
    module = load_loader_module()
    from tf2_ros import Buffer

    loader = SimpleNamespace(tf_buffer=Buffer(), tf_wait_timeout_s=0.01)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "spin_once", lambda *_a, **_kw: None)
    with pytest.raises(module.StaticSceneLoadError, match="world -> base_link"):
        module.StaticSceneLoader.lookup_transform_when_available(
            loader, "base_link", "world"
        )
