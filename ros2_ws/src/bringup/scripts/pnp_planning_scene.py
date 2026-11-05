#!/usr/bin/env python3
"""Validation-only PlanningScene objects and fixture lifecycle policy.

This module deliberately contains no production Motion or GripperPort logic.
The registration handoff is the only fixture pose authority crossing into this
ROS-side validation seam; Isaac prim paths and physics realization stay inside
the Isaac validation runtime.
"""

from __future__ import annotations

import math
import copy
from enum import Enum

from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


class PlanningSceneLifecycleError(RuntimeError):
    """Raised when a fixture lifecycle transition is not safe."""


class LifecycleState(Enum):
    WORLD = "world"
    CAPTURING = "capturing"
    ATTACHED = "attached"
    REMOVED = "removed"


class LifecycleOperation(Enum):
    NONE = "none"
    ATTACH = "attach"
    RETURN_TO_WORLD = "return_to_world"
    REMOVE = "remove"


def fixture_object_id(handoff: dict) -> str:
    return f"wu14_fixture_{handoff['run_id']}_{handoff['target_id']}"


def _pose_from_handoff(handoff: dict) -> Pose:
    """Convert the registered top-surface reference to a box-center pose.

    Fixed Vision and the registration handoff intentionally use the visible
    top-surface centroid.  MoveIt's BOX primitive pose is its geometric
    center, so the center is one half-height along the fixture's local
    negative vertical axis from that reference.  The handoff orientation is
    yaw-only; rotating the local offset through that orientation keeps this
    conversion explicit and prevents the registration contract from being
    mistaken for a primitive-center contract.
    """
    source = handoff["request"]["pose"]
    dimensions = [float(value) for value in handoff["fixture"]["dimensions_m"]]
    if len(dimensions) != 3 or dimensions[2] <= 0.0:
        raise PlanningSceneLifecycleError(
            "fixture dimensions must contain a positive height"
        )

    yaw = float(source["yaw"])
    half_height = dimensions[2] / 2.0
    # The registration contract carries yaw, so the local vertical axis is
    # unchanged by the declared orientation.  Keep the conversion expressed
    # in local coordinates rather than embedding a frame-specific z constant.
    local_center_offset_z = -half_height
    pose = Pose()
    pose.position.x = float(source["x"])
    pose.position.y = float(source["y"])
    pose.position.z = float(source["z"]) + local_center_offset_z
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def make_fixture_collision_object(handoff: dict) -> CollisionObject:
    """Build the world object used before PICK planning.

    The handoff pose is intentionally kept in ``base_link``.  MoveIt accepts
    that frame and resolves it through the existing TF boundary; no Isaac
    scene path or physics attribute is exposed here.
    """

    collision = CollisionObject()
    collision.id = fixture_object_id(handoff)
    collision.header.frame_id = handoff["request"]["pose"]["frame_id"]
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [
        float(value) for value in handoff["fixture"]["dimensions_m"]
    ]
    collision.primitives = [primitive]
    # MoveIt's canonical box representation stores the object transform in
    # CollisionObject.pose; primitive_poses are relative to that object pose.
    collision.pose = _pose_from_handoff(handoff)
    collision.primitive_poses = [Pose()]
    collision.primitive_poses[0].orientation.w = 1.0
    collision.operation = CollisionObject.ADD
    return collision


def make_attached_fixture_object(
    collision: CollisionObject,
    link_name: str,
    touch_links: list[str],
    pose_in_link: Pose | None = None,
) -> AttachedCollisionObject:
    """Convert the fixture object to an attached object after acceptance."""

    if not link_name or not touch_links or pose_in_link is None:
        raise PlanningSceneLifecycleError(
            "attached fixture requires a link, touch links, and a transformed pose"
        )
    attached = AttachedCollisionObject()
    attached.link_name = link_name
    attached.touch_links = list(touch_links)
    attached.object = copy.deepcopy(collision)
    attached.object.header.frame_id = link_name
    attached.object.primitive_poses = [pose_in_link]
    attached.object.operation = CollisionObject.ADD
    return attached


def make_attach_diff(collision: CollisionObject, attached: AttachedCollisionObject):
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    removed = CollisionObject()
    removed.id = collision.id
    removed.operation = CollisionObject.REMOVE
    request.scene.world.collision_objects = [removed]
    request.scene.robot_state.attached_collision_objects = [attached]
    return request


def make_detach_diff(attached: AttachedCollisionObject, world: CollisionObject):
    """Remove the attached object and restore its validation world object."""
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    removed = AttachedCollisionObject()
    removed.link_name = attached.link_name
    removed.object.id = attached.object.id
    removed.object.operation = CollisionObject.REMOVE
    request.scene.robot_state.attached_collision_objects = [removed]
    world.operation = CollisionObject.ADD
    request.scene.world.collision_objects = [world]
    return request


def make_fixture_world_diff(handoff: dict):
    """Build an ApplyPlanningScene request that adds the registered fixture."""

    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.world.collision_objects = [make_fixture_collision_object(handoff)]
    return request


def make_fixture_remove_diff(object_id: str):
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    collision = CollisionObject()
    collision.id = object_id
    collision.operation = CollisionObject.REMOVE
    request.scene.world.collision_objects = [collision]
    return request


class FixturePlanningSceneLifecycle:
    """Fail-closed lifecycle for one validation fixture object."""

    def __init__(self, object_id: str, cleanup_delay_s: float):
        if not object_id or cleanup_delay_s < 0.0:
            raise ValueError("invalid fixture lifecycle configuration")
        self.object_id = object_id
        self.cleanup_delay_s = float(cleanup_delay_s)
        self.state = LifecycleState.WORLD
        self._detached_at = None
        self._cleanup_deadline = None

    def begin_capture(self) -> LifecycleOperation:
        if self.state is not LifecycleState.WORLD:
            raise PlanningSceneLifecycleError("capture must begin from world state")
        self.state = LifecycleState.CAPTURING
        return LifecycleOperation.NONE

    @staticmethod
    def _require_mutation(apply, description):
        if apply is None:
            return
        try:
            applied = bool(apply())
        except Exception as error:
            raise PlanningSceneLifecycleError(
                f"{description} mutation raised: {error}"
            ) from error
        if not applied:
            raise PlanningSceneLifecycleError(f"{description} mutation failed")

    def capture_accepted(self, apply=None) -> LifecycleOperation:
        if self.state is not LifecycleState.CAPTURING:
            raise PlanningSceneLifecycleError(
                "capture acceptance requires a pending capture"
            )
        self._require_mutation(apply, "attach")
        self.state = LifecycleState.ATTACHED
        return LifecycleOperation.ATTACH

    def attachment_failed(self) -> LifecycleOperation:
        if self.state is not LifecycleState.CAPTURING:
            raise PlanningSceneLifecycleError("attachment failure is not pending")
        self.state = LifecycleState.WORLD
        return LifecycleOperation.RETURN_TO_WORLD

    def reconcile_attached(self) -> LifecycleOperation:
        """Accept an authoritative attached realization after observation."""
        if self.state not in (
            LifecycleState.WORLD,
            LifecycleState.CAPTURING,
            LifecycleState.ATTACHED,
        ):
            raise PlanningSceneLifecycleError(
                f"cannot reconcile attached fixture from {self.state.value}"
            )
        self.state = LifecycleState.ATTACHED
        return LifecycleOperation.ATTACH

    def reconcile_world_after_release(self, detached_at: float) -> LifecycleOperation:
        """Record a RELEASED event whose authoritative realization is world-only."""
        if self.state not in (
            LifecycleState.WORLD,
            LifecycleState.CAPTURING,
            LifecycleState.ATTACHED,
        ):
            raise PlanningSceneLifecycleError(
                f"cannot reconcile released fixture from {self.state.value}"
            )
        if self.state is LifecycleState.WORLD and self._detached_at is not None:
            return LifecycleOperation.RETURN_TO_WORLD
        self.state = LifecycleState.WORLD
        self._detached_at = float(detached_at)
        return LifecycleOperation.RETURN_TO_WORLD

    def place_detached(
        self, detached_at: float = 0.0, apply=None
    ) -> LifecycleOperation:
        if self.state is not LifecycleState.ATTACHED:
            raise PlanningSceneLifecycleError("PLACE detach requires attached state")
        self._require_mutation(apply, "world restore")
        self.state = LifecycleState.WORLD
        self._detached_at = float(detached_at)
        return LifecycleOperation.RETURN_TO_WORLD

    def authorize_cleanup(self, authorized_at: float, deadline_at=None) -> bool:
        if self.state is not LifecycleState.WORLD or self._detached_at is None:
            raise PlanningSceneLifecycleError(
                "cleanup authorization requires confirmed detach"
            )
        if float(authorized_at) < self._detached_at:
            raise PlanningSceneLifecycleError("cleanup authorization predates detach")
        self._cleanup_deadline = (
            float(authorized_at) + self.cleanup_delay_s
            if deadline_at is None
            else float(deadline_at)
        )
        if self._cleanup_deadline < self._detached_at:
            raise PlanningSceneLifecycleError(
                "cleanup deadline predates confirmed detach"
            )
        return True

    def cleanup_due(self, now: float) -> bool:
        return (
            self.state is LifecycleState.WORLD
            and self._cleanup_deadline is not None
            and float(now) >= self._cleanup_deadline
        )

    def cleanup(self, apply=None) -> LifecycleOperation:
        if self.state is not LifecycleState.WORLD or self._detached_at is None:
            raise PlanningSceneLifecycleError("fixture cleanup is not ready")
        self._require_mutation(apply, "cleanup remove")
        self.state = LifecycleState.REMOVED
        self._cleanup_deadline = None
        return LifecycleOperation.REMOVE
