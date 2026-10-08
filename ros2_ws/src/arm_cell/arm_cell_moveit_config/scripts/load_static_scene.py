#!/usr/bin/env python3
"""Load and verify the selected bounded static environment in MoveIt."""

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from rclpy.node import Node
from rclpy.duration import Duration
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class StaticSceneValidationError(Exception):
    """Raised when the static-scene artifact violates its schema."""


class StaticSceneLoadError(Exception):
    """Raised when MoveIt cannot apply or verify the static scene."""


def load_artifact(path):
    artifact = json.loads(path.read_text())
    if artifact.get("schema_version") != 1 or artifact.get("frame_id") != "world":
        raise StaticSceneValidationError(
            "static scene artifact has an unsupported schema or frame"
        )
    objects = artifact.get("objects")
    if not isinstance(objects, list) or not objects:
        raise StaticSceneValidationError("static scene artifact has no objects")
    ids = [obj.get("id") for obj in objects]
    if any(not object_id for object_id in ids) or len(ids) != len(set(ids)):
        raise StaticSceneValidationError(
            "static scene artifact has duplicate or empty object ids"
        )
    for obj in objects:
        geometry = obj.get("geometry", {})
        pose = obj.get("pose", {})
        dimensions = geometry.get("dimensions_m")
        position = pose.get("position_m")
        orientation = pose.get("quaternion_xyzw")
        if geometry.get("type") != "box" or len(dimensions or []) != 3:
            raise StaticSceneValidationError(
                f"unsupported geometry for {obj.get('id')}"
            )
        if len(position or []) != 3 or len(orientation or []) != 4:
            raise StaticSceneValidationError(f"invalid pose for {obj.get('id')}")
        if not all(
            math.isfinite(float(value)) and float(value) > 0 for value in dimensions
        ):
            raise StaticSceneValidationError(f"invalid dimensions for {obj['id']}")
        if not all(math.isfinite(float(value)) for value in position + orientation):
            raise StaticSceneValidationError(f"invalid pose values for {obj['id']}")
    return artifact


def make_collision_object(artifact, obj):
    collision = CollisionObject()
    collision.header.frame_id = artifact["frame_id"]
    collision.id = obj["id"]
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in obj["geometry"]["dimensions_m"]]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = obj["pose"]["position_m"]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        obj["pose"]["quaternion_xyzw"]
    )
    collision.pose = pose
    collision.primitives = [primitive]
    collision.primitive_poses = [Pose()]
    collision.primitive_poses[0].orientation.w = 1.0
    collision.operation = CollisionObject.ADD
    return collision


def assert_close(actual, expected, label):
    if any(abs(float(a) - float(e)) > 1e-9 for a, e in zip(actual, expected)):
        raise StaticSceneLoadError(
            f"static scene readback mismatch for {label}: {actual} != {expected}"
        )


class StaticSceneLoader(Node):
    def __init__(self):
        super().__init__("load_static_scene")
        self.declare_parameter("scene_file", "")
        self.declare_parameter("verify_nominal_state", False)
        scene_file = self.get_parameter("scene_file").value
        if not scene_file:
            raise StaticSceneValidationError("scene_file parameter is required")
        self.artifact = load_artifact(Path(scene_file))
        self.declare_parameter("selected_ids", [])
        selected_ids = [
            str(value) for value in self.get_parameter("selected_ids").value
        ]
        if selected_ids:
            selected = set(selected_ids)
            available = {obj["id"] for obj in self.artifact["objects"]}
            if not selected.issubset(available):
                raise StaticSceneValidationError(
                    f"static scene selection contains unknown ids: {sorted(selected - available)}"
                )
            self.artifact = {
                **self.artifact,
                "objects": [
                    obj for obj in self.artifact["objects"] if obj["id"] in selected
                ],
            }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.apply_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.read_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=20.0):
            raise StaticSceneLoadError(f"MoveIt service unavailable: {client.srv_name}")
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        result = future.result()
        if result is None:
            raise StaticSceneLoadError(f"MoveIt service failed: {client.srv_name}")
        return result

    def apply_and_verify(self):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [
            make_collision_object(self.artifact, obj)
            for obj in self.artifact["objects"]
        ]
        response = self.call(self.apply_client, request)
        if not response.success:
            raise StaticSceneLoadError("MoveIt rejected the static planning scene")

        read_request = GetPlanningScene.Request()
        read_request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        scene = self.call(self.read_client, read_request).scene
        actual = {obj.id: obj for obj in scene.world.collision_objects}
        for expected in self.artifact["objects"]:
            object_id = expected["id"]
            if object_id not in actual:
                raise StaticSceneLoadError(
                    f"static scene object missing after apply: {object_id}"
                )
            received = actual[object_id]
            if (
                len(received.primitives) != 1
                or received.primitives[0].type != SolidPrimitive.BOX
            ):
                raise StaticSceneLoadError(
                    f"static scene geometry mismatch for {object_id}"
                )
            assert_close(
                received.primitives[0].dimensions,
                expected["geometry"]["dimensions_m"],
                f"{object_id} dimensions",
            )
            expected_pose = Pose()
            (
                expected_pose.position.x,
                expected_pose.position.y,
                expected_pose.position.z,
            ) = expected["pose"]["position_m"]
            (
                expected_pose.orientation.x,
                expected_pose.orientation.y,
                expected_pose.orientation.z,
                expected_pose.orientation.w,
            ) = expected["pose"]["quaternion_xyzw"]
            transform = self.tf_buffer.lookup_transform(
                received.header.frame_id,
                self.artifact["frame_id"],
                rclpy.time.Time(),
                timeout=Duration(seconds=5.0),
            )
            pose = do_transform_pose(expected_pose, transform)
            actual_pose = received.pose
            assert_close(
                [
                    actual_pose.position.x,
                    actual_pose.position.y,
                    actual_pose.position.z,
                    actual_pose.orientation.x,
                    actual_pose.orientation.y,
                    actual_pose.orientation.z,
                    actual_pose.orientation.w,
                ],
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                f"{object_id} pose",
            )
        verify_nominal = getattr(self, "verify_nominal_state", None)
        if verify_nominal is None:
            parameter = getattr(self, "get_parameter", None)
            verify_nominal = bool(
                parameter("verify_nominal_state").value
                if callable(parameter)
                else False
            )
        if verify_nominal:
            validity_request = GetStateValidity.Request()
            validity_request.robot_state = scene.robot_state
            validity_request.group_name = "arm"
            validity = self.call(self.validity_client, validity_request)
            if not validity.valid:
                raise StaticSceneLoadError(
                    "nominal current robot state is colliding with validation static scene"
                )


def main():
    rclpy.init()
    try:
        node = StaticSceneLoader()
        node.apply_and_verify()
        node.get_logger().info(
            f"Static planning scene applied and verified: {len(node.artifact['objects'])} objects"
        )
        rclpy.spin(node)
        return 0
    except (
        OSError,
        StaticSceneValidationError,
        StaticSceneLoadError,
        json.JSONDecodeError,
    ) as error:
        rclpy.logging.get_logger("load_static_scene").error(
            f"Static scene load failed before or during mutation: {error}"
        )
        return 1
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
