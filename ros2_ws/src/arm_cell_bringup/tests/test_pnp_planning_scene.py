"""Validation-only PlanningScene lifecycle tests for WU-14."""

import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts/pnp_planning_scene.py"


def module():
    spec = importlib.util.spec_from_file_location("pnp_planning_scene", SCRIPT)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def handoff():
    return {
        "run_id": "run-1",
        "target_id": "cube_profile",
        "request": {
            "pose": {
                "frame_id": "base_link",
                "x": 0.2,
                "y": -0.1,
                "z": 0.84,
                "yaw": 0.25,
            }
        },
        "fixture": {"dimensions_m": [0.07, 0.07, 0.08]},
    }


def test_fixture_world_object_converts_registered_top_to_box_center():
    loaded = module()
    fixture_handoff = handoff()
    registered_top_z = fixture_handoff["request"]["pose"]["z"]

    collision = loaded.make_fixture_collision_object(fixture_handoff)

    assert collision.id == "wu14_fixture_run-1_cube_profile"
    assert collision.header.frame_id == "base_link"
    assert collision.primitives[0].dimensions == pytest.approx([0.07, 0.07, 0.08])
    assert collision.pose.position.x == pytest.approx(0.2)
    assert collision.pose.position.y == pytest.approx(-0.1)
    assert collision.pose.position.z == pytest.approx(registered_top_z - 0.04)
    assert collision.primitive_poses[0] == loaded.Pose()
    assert collision.operation == collision.ADD


def test_top_to_center_conversion_preserves_center_for_nonzero_yaw():
    loaded = module()
    fixture_handoff = handoff()
    fixture_handoff["request"]["pose"]["yaw"] = 1.2

    collision = loaded.make_fixture_collision_object(fixture_handoff)
    pose = collision.pose

    assert pose.position.x == pytest.approx(0.2)
    assert pose.position.y == pytest.approx(-0.1)
    assert pose.position.z == pytest.approx(0.80)
    assert pose.orientation.z == pytest.approx(__import__("math").sin(0.6))
    assert pose.orientation.w == pytest.approx(__import__("math").cos(0.6))


def test_registered_top_pose_is_not_mutated_by_planning_scene_conversion():
    loaded = module()
    fixture_handoff = handoff()
    source_pose = dict(fixture_handoff["request"]["pose"])

    loaded.make_fixture_collision_object(fixture_handoff)

    assert fixture_handoff["request"]["pose"] == source_pose


def test_lifecycle_is_fail_closed_and_has_explicit_attachment_transitions():
    loaded = module()
    lifecycle = loaded.FixturePlanningSceneLifecycle(
        "wu14_fixture_run-1_cube_profile", cleanup_delay_s=3.0
    )

    assert lifecycle.state == loaded.LifecycleState.WORLD
    assert lifecycle.begin_capture() == loaded.LifecycleOperation.NONE
    assert lifecycle.capture_accepted() == loaded.LifecycleOperation.ATTACH
    assert lifecycle.state == loaded.LifecycleState.ATTACHED
    assert lifecycle.place_detached() == loaded.LifecycleOperation.RETURN_TO_WORLD
    assert lifecycle.state == loaded.LifecycleState.WORLD
    assert lifecycle.cleanup_due(3.0) is False
    assert lifecycle.authorize_cleanup(10.0) is True
    assert lifecycle.cleanup_due(12.99) is False
    assert lifecycle.cleanup_due(13.0) is True
    assert lifecycle.cleanup() == loaded.LifecycleOperation.REMOVE
    assert lifecycle.state == loaded.LifecycleState.REMOVED

    with pytest.raises(loaded.PlanningSceneLifecycleError):
        lifecycle.capture_accepted()


def test_lifecycle_does_not_claim_capture_when_attach_mutation_fails():
    loaded = module()
    lifecycle = loaded.FixturePlanningSceneLifecycle("fixture", cleanup_delay_s=3.0)
    lifecycle.begin_capture()

    with pytest.raises(loaded.PlanningSceneLifecycleError):
        lifecycle.capture_accepted(apply=lambda: False)
    assert lifecycle.state == loaded.LifecycleState.CAPTURING


def test_lifecycle_does_not_claim_world_restore_or_cleanup_when_mutation_fails():
    loaded = module()
    lifecycle = loaded.FixturePlanningSceneLifecycle("fixture", cleanup_delay_s=3.0)
    lifecycle.begin_capture()
    lifecycle.capture_accepted()

    with pytest.raises(loaded.PlanningSceneLifecycleError):
        lifecycle.place_detached(detached_at=10.0, apply=lambda: False)
    assert lifecycle.state == loaded.LifecycleState.ATTACHED

    lifecycle.place_detached(detached_at=10.0)
    assert lifecycle.cleanup_due(13.0) is False
    with pytest.raises(loaded.PlanningSceneLifecycleError):
        lifecycle.cleanup(apply=lambda: False)
    assert lifecycle.state == loaded.LifecycleState.WORLD


def test_capture_transition_requires_attachment_completion_before_acceptance():
    loaded = module()
    lifecycle = loaded.FixturePlanningSceneLifecycle("fixture", cleanup_delay_s=3.0)

    assert lifecycle.begin_capture() == loaded.LifecycleOperation.NONE
    assert lifecycle.state == loaded.LifecycleState.CAPTURING
    assert lifecycle.attachment_failed() == loaded.LifecycleOperation.RETURN_TO_WORLD
    assert lifecycle.state == loaded.LifecycleState.WORLD


def test_attached_object_requires_explicit_minimal_touch_links():
    loaded = module()
    collision = loaded.make_fixture_collision_object(handoff())

    attached = loaded.make_attached_fixture_object(
        collision,
        "gripper_robotiq_85_base_link",
        [
            "gripper_robotiq_85_left_finger_link",
            "gripper_robotiq_85_right_finger_link",
        ],
        loaded.Pose(),
    )

    assert attached.link_name == "gripper_robotiq_85_base_link"
    assert attached.touch_links == [
        "gripper_robotiq_85_left_finger_link",
        "gripper_robotiq_85_right_finger_link",
    ]
    assert attached.object.id == collision.id

    with pytest.raises(loaded.PlanningSceneLifecycleError):
        loaded.make_attached_fixture_object(collision, "", [], loaded.Pose())

    with pytest.raises(loaded.PlanningSceneLifecycleError):
        loaded.make_attached_fixture_object(
            collision,
            "gripper_robotiq_85_base_link",
            ["gripper_robotiq_85_left_finger_link"],
        )


def test_attach_diff_removes_world_object_and_adds_only_attached_object():
    loaded = module()
    collision = loaded.make_fixture_collision_object(handoff())
    attached = loaded.make_attached_fixture_object(
        collision,
        "gripper_robotiq_85_base_link",
        ["gripper_robotiq_85_left_finger_link"],
        loaded.Pose(),
    )
    request = loaded.make_attach_diff(collision, attached)

    assert request.scene.is_diff is True
    assert request.scene.robot_state.is_diff is True
    assert request.scene.world.collision_objects[0].operation == collision.REMOVE
    assert (
        request.scene.robot_state.attached_collision_objects[0].object.operation
        == collision.ADD
    )
    assert (
        request.scene.robot_state.attached_collision_objects[0].object.id
        == collision.id
    )
    assert request.scene.robot_state.attached_collision_objects[0].touch_links == [
        "gripper_robotiq_85_left_finger_link"
    ]


def test_detach_diff_removes_attached_object_and_adds_world_object_as_diff():
    loaded = module()
    collision = loaded.make_fixture_collision_object(handoff())
    attached = loaded.make_attached_fixture_object(
        collision,
        "gripper_robotiq_85_base_link",
        ["gripper_robotiq_85_left_finger_link"],
        loaded.Pose(),
    )
    world = loaded.make_fixture_collision_object(handoff())
    request = loaded.make_detach_diff(attached, world)

    assert request.scene.is_diff is True
    assert request.scene.robot_state.is_diff is True
    assert request.scene.robot_state.attached_collision_objects[0].object.operation == (
        collision.REMOVE
    )
    assert request.scene.robot_state.attached_collision_objects[0].object.id == (
        collision.id
    )
    assert request.scene.world.collision_objects[0].operation == collision.ADD
    assert request.scene.world.collision_objects[0].id == collision.id
