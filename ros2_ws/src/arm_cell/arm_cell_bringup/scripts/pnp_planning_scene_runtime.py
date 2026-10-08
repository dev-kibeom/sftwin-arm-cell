#!/usr/bin/env python3
"""Validation-only bridge from existing gripper status to PlanningScene diffs."""

from __future__ import annotations

import json
import math
import os
import copy
from pathlib import Path
import tempfile
import threading
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject

from pnp_planning_scene import (
    FixturePlanningSceneLifecycle,
    LifecycleState,
    PlanningSceneLifecycleError,
    make_attached_fixture_object,
    make_attach_diff,
    make_detach_diff,
    make_fixture_collision_object,
    make_fixture_remove_diff,
)


MINIMUM_STATIC_OBJECT_IDS = ("pedestal", "cnc_front_top")
CLEANUP_HANDOFF_SCHEMA_VERSION = "wu14-pnp-cleanup/v1"
PLACE_RESULT_EVENT_SCHEMA_VERSION = "wu14-pnp-place-result/v1"
MISSION_PHASE_PLACING = 4
MISSION_PHASE_RETURNING_HOME = 5
MISSION_PHASE_RETRACTING = 7
MISSION_PHASE_FINISHED = 8
MISSION_COMPLETION_SCHEMA_VERSION = "wu14-pnp-mission-completion/v1"


class PlanningSceneRuntimeError(RuntimeError):
    """Raised when a validation PlanningScene transition cannot be applied."""


SERVICE_CALL_TIMEOUT_SEC = 2.0


def wait_for_service_future(future, timeout_sec):
    """Wait for executor completion without recursively spinning its node."""
    completed = threading.Event()
    future.add_done_callback(lambda _future: completed.set())
    return completed.wait(timeout_sec)


def call_service_with_bounded_wait(
    client, request, operation, context, logger, timeout_sec=SERVICE_CALL_TIMEOUT_SEC
):
    """Call a ROS service and wait without spinning the caller's executor."""
    started_at = time.monotonic()
    logger.info(f"service request start {context}")
    try:
        available = client.wait_for_service(timeout_sec=timeout_sec)
    except Exception as error:
        logger.error(f"service exception {context} error={error}")
        raise PlanningSceneRuntimeError(
            f"{operation} service exception ({context}): {error}"
        ) from error
    if not available:
        logger.error(
            f"service unavailable {context} timeout_sec={timeout_sec:.1f} "
            f"duration_s={time.monotonic() - started_at:.6f}"
        )
        raise PlanningSceneRuntimeError(f"{operation} service unavailable ({context})")
    try:
        future = client.call_async(request)
    except Exception as error:
        logger.error(f"service exception {context} error={error}")
        raise PlanningSceneRuntimeError(
            f"{operation} service exception ({context}): {error}"
        ) from error
    if not wait_for_service_future(future, timeout_sec):
        logger.error(
            f"service response timeout {context} timeout_sec={timeout_sec:.1f} "
            f"duration_s={time.monotonic() - started_at:.6f}"
        )
        raise PlanningSceneRuntimeError(
            f"{operation} service response timeout ({context})"
        )
    try:
        response = future.result()
    except Exception as error:
        logger.error(f"service exception {context} error={error}")
        raise PlanningSceneRuntimeError(
            f"{operation} service exception ({context}): {error}"
        ) from error
    if response is None:
        logger.error(f"empty service response {context}")
        raise PlanningSceneRuntimeError(f"{operation} empty response ({context})")
    logger.info(
        f"service request complete {context} "
        f"duration_s={time.monotonic() - started_at:.6f}"
    )
    return response


class FixtureSceneState:
    WORLD = "WORLD"
    ATTACHED = "ATTACHED"
    ABSENT = "ABSENT"
    INCONSISTENT = "INCONSISTENT"


def _fixture_realization(observation, object_id):
    world = [obj for obj in observation.get("world_objects", []) if obj.id == object_id]
    attached = [
        item
        for item in observation.get("attached_objects", [])
        if item.object.id == object_id
    ]
    if len(world) > 1 or len(attached) > 1 or (world and attached):
        return (
            FixtureSceneState.INCONSISTENT,
            world[0] if world else None,
            attached[0] if attached else None,
        )
    if world:
        return FixtureSceneState.WORLD, world[0], None
    if attached:
        return FixtureSceneState.ATTACHED, None, attached[0]
    return FixtureSceneState.ABSENT, None, None


def _same_world_fixture(actual, expected):
    return (
        actual.id == expected.id
        and actual.header.frame_id == expected.header.frame_id
        and len(actual.primitives) == len(expected.primitives)
        and len(actual.primitive_poses) == len(expected.primitive_poses)
        and all(
            left.type == right.type
            and len(left.dimensions) == len(right.dimensions)
            and all(
                abs(float(a) - float(b)) <= 1e-6
                for a, b in zip(left.dimensions, right.dimensions)
            )
            for left, right in zip(actual.primitives, expected.primitives)
        )
        and _poses_close(actual.pose, expected.pose)
        and all(
            _poses_close(left, right)
            for left, right in zip(actual.primitive_poses, expected.primitive_poses)
        )
    )


def _poses_close(left, right):
    values_left = (
        left.position.x,
        left.position.y,
        left.position.z,
        left.orientation.x,
        left.orientation.y,
        left.orientation.z,
        left.orientation.w,
    )
    values_right = (
        right.position.x,
        right.position.y,
        right.position.z,
        right.orientation.x,
        right.orientation.y,
        right.orientation.z,
        right.orientation.w,
    )
    return all(
        abs(float(a) - float(b)) <= 1e-6 for a, b in zip(values_left, values_right)
    )


def _same_attached_fixture(actual, expected, *, compare_relative_pose=True):
    left, right = actual.object, expected.object
    return (
        actual.link_name == expected.link_name
        and sorted(actual.touch_links) == sorted(expected.touch_links)
        and left.id == right.id
        and left.header.frame_id == right.header.frame_id == expected.link_name
        and len(left.primitives) == len(right.primitives)
        and len(left.primitive_poses) == len(right.primitive_poses)
        and all(
            a.type == b.type
            and len(a.dimensions) == len(b.dimensions)
            and all(
                abs(float(x) - float(y)) <= 1e-6
                for x, y in zip(a.dimensions, b.dimensions)
            )
            for a, b in zip(left.primitives, right.primitives)
        )
        and (
            not compare_relative_pose
            or all(
                _poses_close(a, b)
                for a, b in zip(left.primitive_poses, right.primitive_poses)
            )
        )
    )


def atomic_write_json(path: str | Path, payload: dict) -> Path:
    """Atomically publish a validation-only request across the Python boundary."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return destination


def parse_gripper_status(payload: str) -> dict[str, object]:
    fields = {}
    for field in payload.split(";"):
        key, separator, value = field.partition("=")
        if separator and key in ("attached", "released"):
            fields[key] = value == "1"
    sequence = None
    for field in payload.split(";"):
        key, separator, value = field.partition("=")
        if separator and key == "seq":
            try:
                sequence = int(value)
            except ValueError:
                sequence = None
    return {
        "attached": fields.get("attached", False),
        "released": fields.get("released", False),
        "seq": sequence,
    }


class PlaceResultEventProducer:
    """Observe existing mission feedback and publish a validation-only PLACE fact."""

    def __init__(self, run_id: str, target_id: str, event_path: str | Path):
        self.run_id = run_id
        self.target_id = target_id
        self.event_path = Path(event_path)
        self._goal_id = None
        self._sequence = 0
        self._emitted = False
        self._release_confirmed = False
        self._active_goal_id = None
        self._active_phase = None

    def observe_feedback(
        self,
        goal_id: str,
        phase: int,
        now: float,
        *,
        fresh_released=False,
        detach_confirmed=False,
    ) -> bool:
        if not goal_id:
            return False
        if goal_id != self._goal_id:
            self._goal_id = goal_id
            self._emitted = False
            self._release_confirmed = False
        self._active_goal_id = goal_id
        self._active_phase = int(phase)
        self._release_confirmed = self._release_confirmed or bool(
            fresh_released and detach_confirmed
        )
        return self._emit_if_ready(float(now))

    def observe_release(
        self, now: float, *, fresh_released: bool, detach_confirmed: bool
    ) -> bool:
        self._release_confirmed = self._release_confirmed or bool(
            fresh_released and detach_confirmed
        )
        return self._emit_if_ready(float(now))

    def _emit_if_ready(self, now: float) -> bool:
        # Phase is mission context only. TaskExecutor issues OPEN only after
        # accepted PLACE motion completes; a fresh RELEASED sequence reconciled
        # with PlanningScene WORLD is the release/detach success fact.
        if (
            self._active_goal_id
            and self._active_phase == MISSION_PHASE_PLACING
            and self._release_confirmed
            and not self._emitted
        ):
            self._sequence += 1
            goal_id = self._active_goal_id
            atomic_write_json(
                self.event_path,
                {
                    "schema_version": PLACE_RESULT_EVENT_SCHEMA_VERSION,
                    "status": "SUCCESS",
                    "run_id": self.run_id,
                    "target_id": self.target_id,
                    "result_identity": f"{goal_id}:{self._sequence}",
                    "sequence": self._sequence,
                    "timestamp": now,
                    "release_motion_completed": True,
                    "fresh_released": True,
                    "detach_confirmed": True,
                },
            )
            self._emitted = True
            return True
        if self._active_phase in (MISSION_PHASE_RETRACTING,):
            self._emitted = True
        return False


class MissionCompletionEventProducer:
    """Publish depletion only after the fixture lifecycle reaches REMOVED."""

    def __init__(self, run_id: str, target_id: str, event_path: str | Path):
        self.run_id = run_id
        self.target_id = target_id
        self.event_path = Path(event_path)
        self._finished_goals = set()
        self._depleted_results = set()
        self._scene_absent = False
        self._emitted = False
        self._lock = threading.Lock()

    def observe_feedback(
        self, goal_id: str, phase: int, now: float, scene_absent: bool
    ):
        if not goal_id:
            return False
        with self._lock:
            if int(phase) == MISSION_PHASE_FINISHED:
                self._finished_goals.add(goal_id)
            self._scene_absent = self._scene_absent or bool(scene_absent)
            return self._emit_if_complete(now)

    def observe_scene_absent(self, now: float, scene_absent: bool):
        with self._lock:
            self._scene_absent = self._scene_absent or bool(scene_absent)
            return self._emit_if_complete(now)

    def observe_result(
        self, goal_id: str, action_status: int, exit_reason: int, now: float
    ):
        with self._lock:
            if int(action_status) == 4 and int(exit_reason) == 1 and goal_id:
                self._depleted_results.add(goal_id)
            return self._emit_if_complete(now)

    def _emit_if_complete(self, now: float):
        completed_goals = sorted(self._finished_goals & self._depleted_results)
        if self._emitted or not completed_goals or not self._scene_absent:
            return False
        goal_id = completed_goals[0]
        atomic_write_json(
            self.event_path,
            {
                "schema_version": MISSION_COMPLETION_SCHEMA_VERSION,
                "status": "DEPLETED",
                "run_id": self.run_id,
                "target_id": self.target_id,
                "goal_id": goal_id,
                "go_home_complete": True,
                "scene_absent": True,
                "action_status": 4,
                "exit_reason": "DEPLETED",
                "timestamp": float(now),
            },
        )
        self._emitted = True
        return True


class PlanningSceneRuntimeBridge:
    """Apply lifecycle diffs only after the corresponding service succeeds."""

    def __init__(
        self,
        handoff: dict,
        cleanup_delay_s: float,
        attachment_link="gripper_robotiq_85_base_link",
        touch_links=None,
        logger=None,
    ):
        self.handoff = handoff
        self.collision = make_fixture_collision_object(handoff)
        self.lifecycle = FixturePlanningSceneLifecycle(
            self.collision.id, cleanup_delay_s
        )
        self.attachment_link = attachment_link
        self.touch_links = touch_links or [
            "gripper_robotiq_85_left_finger_link",
            "gripper_robotiq_85_right_finger_link",
        ]
        self.attached = None
        self._last_sequence = 0
        self._last_scene_state = None
        self._scene_fault_latched = False
        self._fault_suppressed_count = 0
        self._fault_suppressed_reported_count = 0
        self._release_observed = False
        self._baseline_release_ignored = False
        self.cleanup_authorization = None
        self.logger = logger

    def _log(self, message):
        if self.logger is not None:
            self.logger.info(message)
        else:
            print(message)

    def _observe(self, observe, source="UNKNOWN", sequence=None):
        if observe is None:
            raise PlanningSceneRuntimeError(
                "PlanningScene realization query is unavailable"
            )
        try:
            observation = observe()
        except Exception as error:
            raise PlanningSceneRuntimeError(
                f"PlanningScene realization query failed: {error}"
            ) from error
        if not isinstance(observation, dict):
            raise PlanningSceneRuntimeError(
                "PlanningScene query returned invalid observation"
            )
        state, world, attached = _fixture_realization(observation, self.collision.id)
        self._log_observation(state, world, attached, source, sequence)
        return state, world, attached

    def _log_observation(self, state, world, attached, source, sequence):
        if state == self._last_scene_state:
            return
        previous_state = self._last_scene_state or "UNKNOWN"
        self._last_scene_state = state
        sequence_value = "-" if sequence is None else str(sequence)
        self._log(
            "fixture scene transition: "
            f"id={self.collision.id} run_id={self.handoff.get('run_id', '-')} "
            f"{previous_state} -> {state} "
            f"world_present={world is not None} "
            f"attached_present={attached is not None} "
            f"timestamp={time.time():.6f} source={source} status_seq={sequence_value}"
        )
        if state == FixtureSceneState.INCONSISTENT and not self._scene_fault_latched:
            self._scene_fault_latched = True
            self._log(
                "fixture scene fault latched: "
                f"id={self.collision.id} run_id={self.handoff.get('run_id', '-')} "
                "state=INCONSISTENT; automatic repair disabled"
            )

    def log_suppressed_fault_events(self):
        if self._fault_suppressed_count <= self._fault_suppressed_reported_count:
            return
        self._log(
            "fixture scene fault suppressed events: "
            f"id={self.collision.id} run_id={self.handoff.get('run_id', '-')} "
            f"suppressed_count={self._fault_suppressed_count}"
        )
        self._fault_suppressed_reported_count = self._fault_suppressed_count

    def _suppress_fault_event(self):
        self._fault_suppressed_count += 1
        if self._fault_suppressed_count == 1:
            self.log_suppressed_fault_events()

    def _validate_world(self, world):
        if world is None or not _same_world_fixture(world, self.collision):
            raise PlanningSceneRuntimeError(
                "world fixture realization metadata mismatch"
            )

    def _expected_attached(self, pose_in_link):
        return make_attached_fixture_object(
            self.collision, self.attachment_link, self.touch_links, pose_in_link
        )

    def _validate_attached(self, actual, expected, *, compare_relative_pose=True):
        if actual is None:
            raise PlanningSceneRuntimeError(
                "attached fixture realization metadata/link/touch-links mismatch"
            )
        if not _same_attached_fixture(actual, expected, compare_relative_pose=False):
            raise PlanningSceneRuntimeError(
                "attached fixture realization metadata/link/touch-links mismatch"
            )
        if compare_relative_pose and not _same_attached_fixture(
            actual, expected, compare_relative_pose=True
        ):
            raise PlanningSceneRuntimeError(
                "attached fixture realization relative pose mismatch"
            )

    def on_status(
        self, payload: str, pose_in_link: Pose | None, apply, now=0.0, observe=None
    ):
        status = parse_gripper_status(payload)
        sequence = status["seq"]
        if sequence is None or int(sequence) <= self._last_sequence:
            return False
        self._last_sequence = int(sequence)
        if self._scene_fault_latched:
            self._suppress_fault_event()
            return False
        if status["released"] and self.lifecycle.state is LifecycleState.REMOVED:
            return False
        if status["attached"]:
            self._baseline_release_ignored = False
        if (
            status["released"]
            and self.lifecycle.state is LifecycleState.WORLD
            and self.attached is None
            and self._baseline_release_ignored
        ):
            # Isaac reports the open gripper as RELEASED from startup and
            # continues publishing it. Check the first baseline against the
            # planning scene, then suppress repeats until a new attachment.
            return False
        if status["attached"]:
            scene_state, world, observed_attached = self._observe(
                observe, "ATTACH_STATUS", sequence
            )
            expected = self._expected_attached(pose_in_link)
            if scene_state == FixtureSceneState.INCONSISTENT:
                raise PlanningSceneRuntimeError(
                    "fixture scene is INCONSISTENT (world and attached)"
                )
            if scene_state == FixtureSceneState.ABSENT:
                raise PlanningSceneRuntimeError("fixture scene is ABSENT during attach")
            if scene_state == FixtureSceneState.ATTACHED:
                if self.attached is not None:
                    self._validate_attached(observed_attached, self.attached)
                else:
                    # On process restart, the observed attached realization is
                    # authoritative. Current TF describes the robot now, not
                    # the link-relative pose captured when attachment occurred.
                    self._validate_attached(
                        observed_attached, expected, compare_relative_pose=False
                    )
                try:
                    self.lifecycle.reconcile_attached()
                except PlanningSceneLifecycleError as error:
                    raise PlanningSceneRuntimeError(str(error)) from error
                self.attached = copy.deepcopy(observed_attached)
                return True
            self._validate_world(world)
            if self.lifecycle.state is LifecycleState.ATTACHED:
                raise PlanningSceneRuntimeError(
                    "runtime ATTACHED but scene realization is WORLD"
                )
            if self.lifecycle.state is LifecycleState.WORLD:
                self.lifecycle.begin_capture()
            elif self.lifecycle.state is not LifecycleState.CAPTURING:
                raise PlanningSceneRuntimeError(
                    f"cannot attach fixture while lifecycle is {self.lifecycle.state.value}"
                )
            request = make_attach_diff(self.collision, expected)
            apply_error = None
            try:
                if not bool(apply(request)):
                    apply_error = "response success=false"
            except PlanningSceneRuntimeError as error:
                apply_error = str(error)
            except Exception as error:
                apply_error = f"service exception: {error}"
            try:
                state_after, world_after, attached_after = self._observe(
                    observe, "ATTACH_STATUS", sequence
                )
                if state_after == FixtureSceneState.ATTACHED:
                    # A successful mutation's scene observation becomes the
                    # canonical realization. If the service reported failure,
                    # reconcile only when the observed object still matches the
                    # representation we requested, including relative pose.
                    self._validate_attached(
                        attached_after,
                        expected,
                        compare_relative_pose=apply_error is not None,
                    )
                    self.lifecycle.reconcile_attached()
                    self.attached = copy.deepcopy(attached_after)
                    if apply_error:
                        self._log(
                            f"attach mutation {apply_error}; reconciled from scene ATTACHED"
                        )
                    return True
                if self.lifecycle.state is LifecycleState.CAPTURING:
                    self.lifecycle.attachment_failed()
                if state_after == FixtureSceneState.WORLD:
                    self._validate_world(world_after)
                    raise PlanningSceneRuntimeError(
                        f"fixture attach failed: {apply_error or 'scene remained WORLD'}; "
                        "post-failure scene=WORLD"
                    )
                raise PlanningSceneRuntimeError(
                    f"fixture attach failed: {apply_error or 'scene did not converge'}; "
                    f"post-failure scene={state_after}"
                )
            except PlanningSceneRuntimeError:
                if self.lifecycle.state is LifecycleState.CAPTURING:
                    self.lifecycle.attachment_failed()
                raise
            except Exception as error:
                if self.lifecycle.state is LifecycleState.CAPTURING:
                    self.lifecycle.attachment_failed()
                raise PlanningSceneRuntimeError(
                    f"fixture attach failed: {apply_error or 'mutation outcome unknown'}; "
                    f"post-failure scene query/validation error: {error}"
                ) from error
            return True

        if status["released"]:
            scene_state, world, observed_attached = self._observe(
                observe, "RELEASE_STATUS", sequence
            )
            if scene_state == FixtureSceneState.INCONSISTENT:
                raise PlanningSceneRuntimeError(
                    "fixture scene is INCONSISTENT (world and attached)"
                )
            if scene_state == FixtureSceneState.ABSENT:
                raise PlanningSceneRuntimeError(
                    "fixture scene is ABSENT during RELEASED"
                )
            if scene_state == FixtureSceneState.WORLD:
                if (
                    self.lifecycle.state is not LifecycleState.ATTACHED
                    and self.attached is None
                    and not self._release_observed
                ):
                    self._baseline_release_ignored = True
                    return False
                self._validate_world(world)
                try:
                    self.lifecycle.reconcile_world_after_release(float(now))
                except PlanningSceneLifecycleError as error:
                    raise PlanningSceneRuntimeError(str(error)) from error
                self.attached = None
                self._release_observed = True
                return True
            if self.lifecycle.state is LifecycleState.REMOVED:
                raise PlanningSceneRuntimeError("cannot release a removed fixture")
            observed_pose = (
                observed_attached.object.primitive_poses[0]
                if observed_attached.object.primitive_poses
                else None
            )
            if observed_pose is None:
                raise PlanningSceneRuntimeError(
                    "attached fixture realization has no primitive pose"
                )
            if self.attached is not None:
                self._validate_attached(observed_attached, self.attached)
            else:
                expected = self._expected_attached(observed_pose)
                self._validate_attached(
                    observed_attached, expected, compare_relative_pose=False
                )
            try:
                self.lifecycle.reconcile_attached()
            except PlanningSceneLifecycleError as error:
                raise PlanningSceneRuntimeError(str(error)) from error
            self.attached = copy.deepcopy(observed_attached)
            request = make_detach_diff(
                self.attached, make_fixture_collision_object(self.handoff)
            )
            apply_error = None
            try:
                if not apply(request):
                    apply_error = "response success=false"
            except PlanningSceneRuntimeError as error:
                apply_error = str(error)
            except Exception as error:
                apply_error = f"service exception: {error}"
            state_after, world_after, attached_after = self._observe(
                observe, "RELEASE_STATUS", sequence
            )
            if state_after != FixtureSceneState.WORLD:
                raise PlanningSceneRuntimeError(
                    f"fixture world restore failed: {apply_error or 'scene did not converge'}; "
                    f"post-release scene={state_after}"
                )
            self._validate_world(world_after)
            try:
                self.lifecycle.reconcile_world_after_release(float(now))
            except PlanningSceneLifecycleError as error:
                raise PlanningSceneRuntimeError(str(error)) from error
            self.attached = None
            self._release_observed = True
            if apply_error:
                self._log(f"detach mutation {apply_error}; reconciled from scene WORLD")
            return True
        return False

    def confirm_place(self, confirmation: dict, now: float) -> bool:
        """Accept one Isaac-owned cleanup decision without rejudging eligibility."""
        if (
            self.lifecycle.state is not LifecycleState.WORLD
            or not self._release_observed
        ):
            return False
        if confirmation.get("schema_version") != CLEANUP_HANDOFF_SCHEMA_VERSION:
            return False
        if confirmation.get("status") != "AUTHORIZED":
            return False
        if confirmation.get("run_id") != self.handoff.get("run_id"):
            return False
        if confirmation.get("target_id") != self.handoff.get("target_id"):
            return False
        if confirmation.get("fixture_prim_identity") != self.handoff["fixture"].get(
            "prim_identity"
        ):
            return False
        decision_id = confirmation.get("decision_id")
        deadline = confirmation.get("cleanup_deadline_s")
        if not isinstance(decision_id, str) or not decision_id:
            return False
        try:
            deadline = float(deadline)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(deadline) or deadline < float(now):
            return False
        try:
            self.lifecycle.authorize_cleanup(float(now), deadline_at=deadline)
        except PlanningSceneLifecycleError:
            return False
        self.cleanup_authorization = dict(confirmation)
        return True

    def on_cleanup_timer(
        self, now: float, apply, write_delete_request, read_delete_ack, observe=None
    ):
        if not self.lifecycle.cleanup_due(now):
            return False
        if self._scene_fault_latched:
            self._suppress_fault_event()
            return False
        ack = read_delete_ack()
        if self._matching_cleanup_ack(ack):
            state, world, attached = self._observe(observe, "CLEANUP")
            if state != FixtureSceneState.ABSENT:
                raise PlanningSceneRuntimeError(
                    f"cleanup ack rejected: final scene={state}, expected ABSENT"
                )
            self.lifecycle.cleanup()
            return True
        try:
            request = make_fixture_remove_diff(self.collision.id)
            apply_error = None
            try:
                if not apply(request):
                    apply_error = "response success=false"
            except PlanningSceneRuntimeError as error:
                apply_error = str(error)
            except Exception as error:
                apply_error = f"service exception: {error}"
            state, world, attached = self._observe(observe, "CLEANUP")
            if state != FixtureSceneState.ABSENT:
                raise PlanningSceneRuntimeError(
                    f"fixture remove failed: {apply_error or 'scene did not converge'}; "
                    f"final scene={state}"
                )
            self._log(
                f"fixture cleanup convergence: id={self.collision.id} scene=ABSENT"
            )
            if apply_error:
                self._log(
                    f"fixture remove mutation {apply_error}; reconciled from scene ABSENT"
                )
            write_delete_request(self._delete_request())
        except PlanningSceneRuntimeError:
            raise
        except (PlanningSceneLifecycleError, Exception) as error:
            raise PlanningSceneRuntimeError(
                f"fixture cleanup failed: {error}"
            ) from error
        return False

    def _delete_request(self):
        return {
            "schema_version": CLEANUP_HANDOFF_SCHEMA_VERSION,
            "status": "DELETE_REQUESTED",
            "run_id": self.cleanup_authorization["run_id"],
            "target_id": self.cleanup_authorization["target_id"],
            "fixture_prim_identity": self.cleanup_authorization[
                "fixture_prim_identity"
            ],
            "decision_id": self.cleanup_authorization["decision_id"],
            "fresh_released": self._release_observed,
            "detach_confirmed": self._release_observed,
            "ownership_unambiguous": True,
            "scene_absent_confirmed": True,
        }

    def _matching_cleanup_ack(self, ack):
        return (
            isinstance(ack, dict)
            and all(
                ack.get(key) == self.cleanup_authorization.get(key)
                for key in (
                    "schema_version",
                    "run_id",
                    "target_id",
                    "fixture_prim_identity",
                    "decision_id",
                )
            )
            and ack.get("status") in ("PHYSICAL_DELETE_PENDING", "DELETED")
        )


class PlanningSceneRuntimeNode:
    """ROS wrapper; production nodes only see the existing status topic."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node

        from moveit_msgs.srv import ApplyPlanningScene
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        from arm_cell_interfaces.action import ExecuteCycle
        from action_msgs.msg import GoalStatusArray
        from rclpy.callback_groups import (
            MutuallyExclusiveCallbackGroup,
            ReentrantCallbackGroup,
        )
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from tf2_geometry_msgs import do_transform_pose
        from tf2_ros import Buffer, TransformListener

        class _Node(Node):
            def __init__(self):
                super().__init__("pnp_planning_scene_runtime")
                self.declare_parameter(
                    "state_file", "/tmp/sftwin_pnp_validation/planning_scene_state.json"
                )
                self.declare_parameter(
                    "place_confirmation_file",
                    "/tmp/sftwin_pnp_validation/place_confirmation.json",
                )
                self.declare_parameter(
                    "place_result_event_file",
                    "/tmp/sftwin_pnp_validation/place_result_event.json",
                )
                self.declare_parameter(
                    "mission_completion_event_file",
                    "/tmp/sftwin_pnp_validation/mission_completion_event.json",
                )
                self.declare_parameter(
                    "fixture_delete_request_file",
                    "/tmp/sftwin_pnp_validation/fixture_delete_request.json",
                )
                self.declare_parameter(
                    "fixture_delete_ack_file",
                    "/tmp/sftwin_pnp_validation/fixture_delete_ack.json",
                )
                self.declare_parameter("cleanup_delay_s", 3.0)
                self.declare_parameter(
                    "attachment_link", "gripper_robotiq_85_base_link"
                )
                self.declare_parameter(
                    "touch_links",
                    [
                        "gripper_robotiq_85_left_finger_link",
                        "gripper_robotiq_85_right_finger_link",
                    ],
                )
                self.state_file = Path(self.get_parameter("state_file").value)
                self.place_confirmation_file = Path(
                    self.get_parameter("place_confirmation_file").value
                )
                self.place_result_event_file = Path(
                    self.get_parameter("place_result_event_file").value
                )
                self.mission_completion_event_file = Path(
                    self.get_parameter("mission_completion_event_file").value
                )
                self.fixture_delete_request_file = Path(
                    self.get_parameter("fixture_delete_request_file").value
                )
                self.fixture_delete_ack_file = Path(
                    self.get_parameter("fixture_delete_ack_file").value
                )
                self.bridge = None
                self._service_callback_group = ReentrantCallbackGroup()
                self._lifecycle_callback_group = MutuallyExclusiveCallbackGroup()
                self._service_request_lock = threading.Lock()
                self._service_request_id = 0
                self._mission_result_requests = set()
                self.place_event_producer = None
                self.mission_completion_producer = None
                self._state_mtime = None
                self._place_confirmation_mtime = None
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.apply_client = self.create_client(
                    ApplyPlanningScene,
                    "/apply_planning_scene",
                    callback_group=self._service_callback_group,
                )
                self.scene_client = self.create_client(
                    GetPlanningScene,
                    "/get_planning_scene",
                    callback_group=self._service_callback_group,
                )
                self.mission_result_client = self.create_client(
                    ExecuteCycle.Impl.GetResultService,
                    "/orchestration/execute_cycle/_action/get_result",
                    callback_group=self._service_callback_group,
                )
                self.fixture_object_id_publisher = self.create_publisher(
                    String,
                    "/pnp/current_fixture_object_id",
                    QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    ),
                )
                self.create_subscription(
                    String,
                    "/gripper/status",
                    self._status,
                    10,
                    callback_group=self._lifecycle_callback_group,
                )
                self.create_subscription(
                    ExecuteCycle.Impl.FeedbackMessage,
                    "/orchestration/execute_cycle/_action/feedback",
                    self._cycle_feedback,
                    10,
                    callback_group=self._lifecycle_callback_group,
                )
                self.create_subscription(
                    GoalStatusArray,
                    "/orchestration/execute_cycle/_action/status",
                    self._cycle_status,
                    10,
                    callback_group=self._lifecycle_callback_group,
                )
                self.create_timer(0.1, self._poll_state)
                self.create_timer(0.1, self._poll_place_confirmation)
                self.create_timer(
                    0.1,
                    self._cleanup,
                    callback_group=self._lifecycle_callback_group,
                )

            def _poll_state(self):
                if not self.state_file.is_file():
                    return
                mtime = self.state_file.stat().st_mtime_ns
                if mtime == self._state_mtime:
                    return
                handoff = json.loads(self.state_file.read_text())
                next_collision = make_fixture_collision_object(handoff)
                if self.bridge is not None:
                    if self.bridge.collision.id == next_collision.id:
                        self._state_mtime = mtime
                        return
                    self.bridge.log_suppressed_fault_events()
                self.bridge = PlanningSceneRuntimeBridge(
                    handoff,
                    float(self.get_parameter("cleanup_delay_s").value),
                    str(self.get_parameter("attachment_link").value),
                    list(self.get_parameter("touch_links").value),
                    self.get_logger(),
                )
                fixture_object_id = String()
                fixture_object_id.data = self.bridge.collision.id
                self.fixture_object_id_publisher.publish(fixture_object_id)
                self.place_event_producer = PlaceResultEventProducer(
                    handoff["run_id"],
                    handoff["target_id"],
                    self.place_result_event_file,
                )
                self.mission_completion_producer = MissionCompletionEventProducer(
                    handoff["run_id"],
                    handoff["target_id"],
                    self.mission_completion_event_file,
                )
                self._state_mtime = mtime

            def _cycle_feedback(self, message):
                if self.place_event_producer is None:
                    return
                goal_id = bytes(message.goal_id.uuid).hex()
                phase = int(message.feedback.phase.value)
                now = time.time()
                self.place_event_producer.observe_feedback(
                    goal_id,
                    phase,
                    now,
                    fresh_released=(
                        self.bridge is not None and self.bridge._release_observed
                    ),
                    detach_confirmed=(
                        self.bridge is not None
                        and self.bridge.lifecycle.state is LifecycleState.WORLD
                    ),
                )
                if self.mission_completion_producer is not None:
                    self.mission_completion_producer.observe_feedback(
                        goal_id,
                        phase,
                        now,
                        self.bridge is not None
                        and self.bridge.lifecycle.state is LifecycleState.REMOVED,
                    )

            def _cycle_status(self, message):
                producer = self.mission_completion_producer
                if (
                    producer is None
                    or not self.mission_result_client.service_is_ready()
                ):
                    return
                for status in message.status_list:
                    if int(status.status) != 4:
                        continue
                    goal_id = bytes(status.goal_info.goal_id.uuid).hex()
                    if not goal_id or goal_id in self._mission_result_requests:
                        continue
                    self._mission_result_requests.add(goal_id)
                    request = ExecuteCycle.Impl.GetResultService.Request()
                    request.goal_id = status.goal_info.goal_id
                    future = self.mission_result_client.call_async(request)

                    def _result_done(done_future, requested_goal_id=goal_id):
                        try:
                            response = done_future.result()
                            producer.observe_result(
                                requested_goal_id,
                                int(response.status),
                                int(response.result.exit_reason.value),
                                time.time(),
                            )
                        except Exception as error:
                            self.get_logger().warning(
                                f"ExecuteCycle result query failed for goal "
                                f"{requested_goal_id}: {error}"
                            )
                            self._mission_result_requests.discard(requested_goal_id)

                    future.add_done_callback(_result_done)

            def _poll_place_confirmation(self):
                if self.bridge is None or not self.place_confirmation_file.is_file():
                    return
                mtime = self.place_confirmation_file.stat().st_mtime_ns
                if mtime == self._place_confirmation_mtime:
                    return
                try:
                    confirmation = json.loads(self.place_confirmation_file.read_text())
                except (OSError, json.JSONDecodeError) as error:
                    self.get_logger().error(
                        f"invalid place confirmation ignored: {error}"
                    )
                    return
                self.bridge.confirm_place(
                    confirmation,
                    time.time(),
                )
                self._place_confirmation_mtime = mtime

            def _next_service_context(self, source, sequence, operation):
                with self._service_request_lock:
                    self._service_request_id += 1
                    request_id = self._service_request_id
                sequence_value = "-" if sequence is None else str(sequence)
                return (
                    f"source={source} seq={sequence_value} "
                    f"request_id={request_id} service={operation}"
                )

            def _apply(self, request, source="UNKNOWN", sequence=None):
                context = self._next_service_context(
                    source, sequence, "ApplyPlanningScene"
                )
                response = call_service_with_bounded_wait(
                    self.apply_client,
                    request,
                    "ApplyPlanningScene",
                    context,
                    self.get_logger(),
                )
                if not response.success:
                    self.get_logger().error(
                        f"ApplyPlanningScene response success=false ({context})"
                    )
                    raise PlanningSceneRuntimeError(
                        f"ApplyPlanningScene response success=false ({context})"
                    )
                return True

            def _query_fixture_scene(self, source="UNKNOWN", sequence=None):
                context = self._next_service_context(
                    source, sequence, "GetPlanningScene"
                )
                request = GetPlanningScene.Request()
                request.components.components = (
                    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                    | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                )
                response = call_service_with_bounded_wait(
                    self.scene_client,
                    request,
                    "GetPlanningScene",
                    context,
                    self.get_logger(),
                )
                return {
                    "world_objects": response.scene.world.collision_objects,
                    "attached_objects": response.scene.robot_state.attached_collision_objects,
                }

            def _write_delete_request(self, request):
                atomic_write_json(self.fixture_delete_request_file, request)

            def _read_delete_ack(self):
                if not self.fixture_delete_ack_file.is_file():
                    return None
                try:
                    return json.loads(self.fixture_delete_ack_file.read_text())
                except (OSError, json.JSONDecodeError) as error:
                    self.get_logger().error(
                        f"invalid fixture delete ack ignored: {error}"
                    )
                    return None

            def _status(self, message):
                if self.bridge is None:
                    return
                status = parse_gripper_status(message.data)
                source = (
                    "ATTACH_STATUS"
                    if status["attached"]
                    else "RELEASE_STATUS" if status["released"] else "STATUS"
                )
                sequence = status["seq"]
                pose = None
                if status["attached"]:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            self.bridge.attachment_link,
                            self.bridge.collision.header.frame_id,
                            rclpy.time.Time(),
                        )
                        pose = do_transform_pose(
                            self.bridge.collision.primitive_poses[0], transform
                        )
                    except Exception as error:
                        self.get_logger().error(
                            f"fixture attach transform failed: {error}"
                        )
                        return
                try:
                    changed = self.bridge.on_status(
                        message.data,
                        pose,
                        lambda request: self._apply(request, source, sequence),
                        time.time(),
                        lambda: self._query_fixture_scene(source, sequence),
                    )
                    status = parse_gripper_status(message.data)
                    if (
                        changed
                        and status["released"]
                        and self.place_event_producer is not None
                    ):
                        self.place_event_producer.observe_release(
                            time.time(),
                            fresh_released=self.bridge._release_observed,
                            detach_confirmed=(
                                self.bridge.lifecycle.state is LifecycleState.WORLD
                            ),
                        )
                except PlanningSceneRuntimeError as error:
                    self.get_logger().error(str(error))

            def _cleanup(self):
                if self.bridge is None:
                    return
                try:
                    self.bridge.on_cleanup_timer(
                        time.time(),
                        lambda request: self._apply(request, "CLEANUP"),
                        self._write_delete_request,
                        self._read_delete_ack,
                        lambda: self._query_fixture_scene("CLEANUP"),
                    )
                    if self.mission_completion_producer is not None:
                        self.mission_completion_producer.observe_scene_absent(
                            time.time(),
                            self.bridge.lifecycle.state is LifecycleState.REMOVED,
                        )
                except PlanningSceneRuntimeError as error:
                    self.get_logger().error(str(error))

        self.node = _Node()


def main():
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    wrapper = PlanningSceneRuntimeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(wrapper.node)
    try:
        executor.spin()
    finally:
        executor.remove_node(wrapper.node)
        executor.shutdown()
        wrapper.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
