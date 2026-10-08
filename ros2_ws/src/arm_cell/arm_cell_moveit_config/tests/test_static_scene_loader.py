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
                    "position_m": [1.0, 2.0, 3.0],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
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
