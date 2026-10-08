"""Runtime event bridge tests for validation-only PlanningScene lifecycle."""

import importlib.util
import ast
import copy
import json
from pathlib import Path
import sys
import threading
import time
import uuid

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts/pnp_planning_scene_runtime.py"


class FakeFuture:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def complete(self):
        for callback in self.callbacks:
            callback(self)


class FakeServiceClient:
    def __init__(self, *, available=True, future=None):
        self.available = available
        self.future = future or FakeFuture()

    def wait_for_service(self, timeout_sec):
        return self.available

    def call_async(self, request):
        return self.future


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)


def module():
    sys.path.insert(0, str(SCRIPT.parent))
    spec = importlib.util.spec_from_file_location("pnp_planning_scene_runtime", SCRIPT)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def test_service_future_wait_uses_completion_event_with_bounded_timeout():
    loaded = module()
    future = FakeFuture()
    threading.Thread(target=lambda: (time.sleep(0.01), future.complete())).start()

    assert loaded.wait_for_service_future(future, timeout_sec=0.5)
    assert not loaded.wait_for_service_future(FakeFuture(), timeout_sec=0.01)


def test_service_unavailable_and_response_timeout_have_distinct_errors_and_context():
    loaded = module()
    logger = FakeLogger()
    with pytest.raises(loaded.PlanningSceneRuntimeError, match="service unavailable"):
        loaded.call_service_with_bounded_wait(
            FakeServiceClient(available=False),
            object(),
            "GetPlanningScene",
            "source=CLEANUP request_id=10",
            logger,
            timeout_sec=0.01,
        )

    with pytest.raises(
        loaded.PlanningSceneRuntimeError, match="service response timeout"
    ):
        loaded.call_service_with_bounded_wait(
            FakeServiceClient(),
            object(),
            "GetPlanningScene",
            "source=ATTACH_STATUS seq=4 request_id=11",
            logger,
            timeout_sec=0.01,
        )
    assert any("source=CLEANUP" in message for message in logger.messages)
    assert any("source=ATTACH_STATUS seq=4" in message for message in logger.messages)


def test_ros_service_adapters_and_main_avoid_nested_spin():
    tree = ast.parse(SCRIPT.read_text())
    adapter_methods = {
        node.name: node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name in {"_apply", "_query_fixture_scene"}
    }
    assert set(adapter_methods) == {"_apply", "_query_fixture_scene"}
    forbidden = {"spin", "spin_once", "spin_until_future_complete"}
    for method in adapter_methods.values():
        assert not any(
            isinstance(node, ast.Attribute) and node.attr in forbidden
            for node in ast.walk(method)
        )


@pytest.mark.parametrize(
    "source,operation",
    [
        ("ATTACH_STATUS", "GetPlanningScene"),
        ("CLEANUP", "GetPlanningScene"),
        ("ATTACH_STATUS", "ApplyPlanningScene"),
    ],
)
def test_multithreaded_executor_completes_service_response_during_callback_wait(
    source, operation
):
    pytest.importorskip("rclpy")
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    loaded = module()
    if not rclpy.ok():
        rclpy.init()
    suffix = uuid.uuid4().hex
    service_name = f"/bounded_wait_{suffix}"
    server = Node(f"bounded_wait_server_{suffix}")
    client = Node(f"bounded_wait_client_{suffix}")
    callback_group = ReentrantCallbackGroup()
    server.create_service(
        Trigger,
        service_name,
        lambda request, response: _trigger_response(response),
    )
    client_api = client.create_client(
        Trigger, service_name, callback_group=callback_group
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    executor.add_node(client)
    callback_result = {}
    callback_finished = threading.Event()

    def status_or_cleanup_callback():
        request = Trigger.Request()
        request_id = f"{source}:{operation}:{suffix}"
        future = client_api.call_async(request)
        callback_result["completed"] = loaded.wait_for_service_future(
            future, timeout_sec=1.0
        )
        if callback_result["completed"]:
            callback_result["response"] = future.result().success
        callback_result["request_id"] = request_id
        callback_finished.set()

    timer = client.create_timer(0.02, status_or_cleanup_callback)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert callback_finished.wait(2.0), f"callback timed out: {source} {operation}"
        assert callback_result["completed"]
        assert callback_result["response"]
        assert callback_result["request_id"].startswith(f"{source}:{operation}:")
    finally:
        timer.cancel()
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        executor.remove_node(client)
        executor.remove_node(server)
        client.destroy_node()
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_executor_serializes_duplicate_status_transition_once():
    pytest.importorskip("rclpy")
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    loaded = module()
    if not rclpy.ok():
        rclpy.init()
    suffix = uuid.uuid4().hex
    node = Node(f"duplicate_status_{suffix}")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    callback_group = MutuallyExclusiveCallbackGroup()
    bridge, _, requests, apply, observe = configured(loaded)
    results = []
    finished = threading.Event()
    timers = []

    def callback_for(timer_index):
        def callback():
            results.append(
                bridge.on_status(
                    "attached=1;released=0;seq=1",
                    loaded.Pose(),
                    apply,
                    observe=observe,
                )
            )
            timers[timer_index].cancel()
            if len(results) == 2:
                finished.set()

        return callback

    timers.extend(
        [
            node.create_timer(
                0.02,
                callback_for(index),
                callback_group=callback_group,
            )
            for index in range(2)
        ]
    )
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert finished.wait(2.0)
        assert sorted(results) == [False, True]
        assert len(requests) == 1
        assert bridge.lifecycle.state == loaded.LifecycleState.ATTACHED
    finally:
        for timer in timers:
            timer.cancel()
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _trigger_response(response):
    response.success = True
    response.message = "complete"
    return response


def handoff():
    return {
        "schema_version": "wu14-pnp-registration/v1",
        "status": "FIXTURE_READY_FOR_REGISTRATION",
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
        "fixture": {
            "prim_identity": "/World/SFTwin/ValidationFixture/Cube",
            "dimensions_m": [0.07, 0.07, 0.08],
        },
    }


def confirmation():
    return {
        "schema_version": "wu14-pnp-cleanup/v1",
        "status": "AUTHORIZED",
        "run_id": "run-1",
        "target_id": "cube_profile",
        "fixture_prim_identity": "/World/SFTwin/ValidationFixture/Cube",
        "decision_id": "decision-1",
        "cleanup_deadline_s": 13.0,
        "pose": {"x": 0.2, "y": -0.1, "z": 0.84},
    }


def scene_harness(loaded, bridge):
    scene = {"world_objects": [copy.deepcopy(bridge.collision)], "attached_objects": []}
    requests = []

    def apply(request):
        requests.append(request)
        for obj in request.scene.world.collision_objects:
            if obj.operation == loaded.CollisionObject.REMOVE:
                scene["world_objects"] = [
                    x for x in scene["world_objects"] if x.id != obj.id
                ]
            else:
                scene["world_objects"] = [
                    x for x in scene["world_objects"] if x.id != obj.id
                ]
                scene["world_objects"].append(copy.deepcopy(obj))
        for item in request.scene.robot_state.attached_collision_objects:
            if item.object.operation == loaded.CollisionObject.REMOVE:
                scene["attached_objects"] = [
                    x
                    for x in scene["attached_objects"]
                    if x.object.id != item.object.id
                ]
            else:
                scene["attached_objects"] = [
                    x
                    for x in scene["attached_objects"]
                    if x.object.id != item.object.id
                ]
                scene["attached_objects"].append(copy.deepcopy(item))
        return True

    return scene, requests, apply, lambda: scene


def configured(loaded):
    bridge = loaded.PlanningSceneRuntimeBridge(handoff(), cleanup_delay_s=3.0)
    scene, requests, apply, observe = scene_harness(loaded, bridge)
    return bridge, scene, requests, apply, observe


def test_status_parser_uses_existing_gripper_status_fields():
    loaded = module()

    assert loaded.parse_gripper_status(
        "ready=1;grasp=1;attached=1;released=0;width_mm=70;seq=4"
    ) == {"attached": True, "released": False, "seq": 4}


def test_mission_completion_event_waits_for_depleted_result_and_scene_absent(
    tmp_path,
):
    loaded = module()
    event_path = tmp_path / "mission_completion_event.json"
    producer = loaded.MissionCompletionEventProducer(
        "run-42", "cube_profile", event_path
    )

    assert not producer.observe_feedback(
        "goal-1", loaded.MISSION_PHASE_FINISHED, 1.0, False
    )
    assert not producer.observe_result("goal-1", 4, 2, 1.5)
    assert not event_path.exists()
    assert not producer.observe_result("goal-1", 4, 1, 1.75)
    assert not event_path.exists()
    assert producer.observe_scene_absent(2.0, True)
    assert not producer.observe_result("goal-1", 4, 1, 2.5)
    assert not producer.observe_scene_absent(3.0, True)

    event = json.loads(event_path.read_text())
    assert event["status"] == "DEPLETED"
    assert event["go_home_complete"] is True
    assert event["scene_absent"] is True
    assert event["goal_id"] == "goal-1"


def test_scene_transition_logs_only_on_state_change_with_status_context():
    loaded = module()
    logger = FakeLogger()
    bridge = loaded.PlanningSceneRuntimeBridge(
        handoff(), cleanup_delay_s=3.0, logger=logger
    )
    scene, _, apply, observe = scene_harness(loaded, bridge)
    bridge.on_status("attached=1;seq=1", loaded.Pose(), apply, observe=observe)
    transition_logs = [
        message for message in logger.messages if "fixture scene transition:" in message
    ]
    assert len(transition_logs) == 2
    assert "UNKNOWN -> WORLD" in transition_logs[0]
    assert "source=ATTACH_STATUS" in transition_logs[0]
    assert "WORLD -> ATTACHED" in transition_logs[1]
    assert "status_seq=1" in transition_logs[1]

    bridge.on_status("attached=1;seq=2", loaded.Pose(), apply, observe=observe)
    transition_logs_after_duplicate = [
        message for message in logger.messages if "fixture scene transition:" in message
    ]
    assert len(transition_logs_after_duplicate) == 2


def test_inconsistent_scene_latches_fault_and_suppresses_lifecycle_mutations():
    loaded = module()
    logger = FakeLogger()
    bridge = loaded.PlanningSceneRuntimeBridge(
        handoff(), cleanup_delay_s=3.0, logger=logger
    )
    scene, _, _, base_observe = scene_harness(loaded, bridge)
    query_count = 0

    def observe():
        nonlocal query_count
        query_count += 1
        return base_observe()

    apply_calls = []

    def attach_leaves_both_realizations(request):
        apply_calls.append(request)
        scene["attached_objects"].append(
            copy.deepcopy(request.scene.robot_state.attached_collision_objects[0])
        )
        return False

    with pytest.raises(loaded.PlanningSceneRuntimeError, match="INCONSISTENT"):
        bridge.on_status(
            "attached=1;released=0;seq=1",
            loaded.Pose(),
            attach_leaves_both_realizations,
            observe=observe,
        )

    assert bridge._scene_fault_latched
    assert query_count == 2
    assert len(apply_calls) == 1

    assert not bridge.on_status(
        "attached=1;released=0;seq=2",
        loaded.Pose(),
        lambda _: pytest.fail(),
        observe=observe,
    )
    assert not bridge.on_status(
        "attached=0;released=1;seq=3",
        None,
        lambda _: pytest.fail(),
        observe=observe,
    )
    bridge.lifecycle.reconcile_world_after_release(4.0)
    bridge.lifecycle.authorize_cleanup(4.0, deadline_at=5.0)
    assert not bridge.on_cleanup_timer(
        5.0,
        lambda _: pytest.fail(),
        lambda _: pytest.fail(),
        lambda: pytest.fail(),
        observe,
    )
    assert query_count == 2
    assert len(apply_calls) == 1
    bridge.log_suppressed_fault_events()
    assert any("suppressed_count=3" in message for message in logger.messages)

    next_handoff = handoff()
    next_handoff["run_id"] = "run-2"
    next_run = loaded.PlanningSceneRuntimeBridge(next_handoff, cleanup_delay_s=3.0)
    assert not next_run._scene_fault_latched


def test_runtime_bridge_executes_world_attach_restore_remove_sequence():
    loaded = module()
    handoff_data = handoff()
    bridge = loaded.PlanningSceneRuntimeBridge(handoff_data, cleanup_delay_s=3.0)
    scene, requests, apply, observe = scene_harness(loaded, bridge)

    assert (
        bridge.on_status(
            "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
        )
        is True
    )
    assert bridge.lifecycle.state == loaded.LifecycleState.ATTACHED
    assert (
        requests[0].scene.world.collision_objects[0].operation
        == loaded.CollisionObject.REMOVE
    )

    assert (
        bridge.on_status(
            "attached=0;released=1;seq=2", None, apply, now=10.0, observe=observe
        )
        is True
    )
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD
    assert (
        requests[1].scene.world.collision_objects[0].operation
        == loaded.CollisionObject.ADD
    )

    assert (
        bridge.on_cleanup_timer(13.0, apply, lambda _: True, lambda: None, observe)
        is False
    )
    assert bridge.confirm_place(confirmation(), now=10.0) is True
    requests_written = []
    acks = []
    assert (
        bridge.on_cleanup_timer(
            13.0,
            apply,
            requests_written.append,
            lambda: acks[-1] if acks else None,
            observe,
        )
        is False
    )
    assert requests_written[0]["decision_id"] == "decision-1"
    acks.append({**confirmation(), "status": "PHYSICAL_DELETE_PENDING"})
    assert (
        bridge.on_cleanup_timer(
            13.0,
            apply,
            requests_written.append,
            lambda: acks[-1],
            observe,
        )
        is True
    )
    assert bridge.lifecycle.state == loaded.LifecycleState.REMOVED
    assert requests_written[0]["scene_absent_confirmed"] is True
    assert (
        requests[2].scene.world.collision_objects[0].operation
        == loaded.CollisionObject.REMOVE
    )


def test_cleanup_uses_one_validated_decision_for_planning_scene_and_fixture_delete():
    loaded = module()
    bridge = loaded.PlanningSceneRuntimeBridge(handoff(), cleanup_delay_s=3.0)
    _, _, apply, observe = scene_harness(loaded, bridge)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    bridge.on_status(
        "attached=0;released=1;seq=2", None, apply, now=5.0, observe=observe
    )
    bridge.confirm_place({**confirmation(), "cleanup_deadline_s": 8.0}, now=5.0)
    requests = []
    assert (
        bridge.on_cleanup_timer(8.0, apply, requests.append, lambda: None, observe)
        is False
    )
    assert requests[0]["decision_id"] == bridge.cleanup_authorization["decision_id"]


def test_removed_ignores_repeated_released_without_scene_query_or_mutation():
    loaded = module()
    logger = FakeLogger()
    bridge = loaded.PlanningSceneRuntimeBridge(
        handoff(), cleanup_delay_s=3.0, logger=logger
    )
    scene, requests, apply, base_observe = scene_harness(loaded, bridge)
    query_count = 0

    def observe():
        nonlocal query_count
        query_count += 1
        return base_observe()

    bridge.on_status("attached=1;seq=1", loaded.Pose(), apply, observe=observe)
    bridge.on_status("released=1;seq=2", None, apply, now=5.0, observe=observe)
    bridge.confirm_place({**confirmation(), "cleanup_deadline_s": 8.0}, now=5.0)
    written = []
    bridge.on_cleanup_timer(8.0, apply, written.append, lambda: None, observe)
    ack = {**confirmation(), "status": "PHYSICAL_DELETE_PENDING"}
    bridge.on_cleanup_timer(8.0, apply, written.append, lambda: ack, observe)
    assert bridge.lifecycle.state == loaded.LifecycleState.REMOVED
    query_count_after_removal = query_count
    mutation_count_after_removal = len(requests)

    assert not bridge.on_status("released=1;seq=3", None, apply, observe=observe)
    assert not bridge.on_status(
        "attached=0;released=1;seq=4", None, apply, observe=observe
    )
    assert query_count == query_count_after_removal
    assert len(requests) == mutation_count_after_removal
    assert not any("ABSENT during RELEASED" in item for item in logger.messages)


def test_initial_released_status_is_ignored_until_fixture_was_attached():
    loaded = module()
    logger = FakeLogger()
    bridge = loaded.PlanningSceneRuntimeBridge(
        handoff(), cleanup_delay_s=3.0, logger=logger
    )
    scene, requests, apply, base_observe = scene_harness(loaded, bridge)
    queries = 0

    def observe():
        nonlocal queries
        queries += 1
        return base_observe()

    assert not bridge.on_status(
        "attached=0;released=1;seq=1", None, apply, now=1.0, observe=observe
    )
    assert not bridge.on_status(
        "attached=0;released=1;seq=2", None, apply, now=1.1, observe=observe
    )
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD
    assert bridge.attached is None
    assert not bridge._release_observed
    assert queries == 1
    assert requests == []
    assert [x.id for x in scene["world_objects"]] == [bridge.collision.id]
    assert not scene["attached_objects"]
    assert len(logger.messages) == 1
    assert "UNKNOWN -> WORLD" in logger.messages[0]


def test_attach_apply_failure_with_world_scene_rolls_lifecycle_back():
    loaded = module()
    bridge, _, _, _, observe = configured(loaded)

    with pytest.raises(
        loaded.PlanningSceneRuntimeError, match="post-failure scene=WORLD"
    ):
        bridge.on_status(
            "attached=1;released=0;seq=1",
            loaded.Pose(),
            lambda _: False,
            observe=observe,
        )
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD


def test_failed_attach_response_reconciles_attached_post_observation():
    loaded = module()
    bridge, scene, requests, _, observe = configured(loaded)
    apply_calls = []

    def failed_response_after_apply(request):
        apply_calls.append(request)
        scene["world_objects"].clear()
        scene["attached_objects"].append(
            copy.deepcopy(request.scene.robot_state.attached_collision_objects[0])
        )
        return False

    assert bridge.on_status(
        "attached=1;released=0;seq=1",
        loaded.Pose(),
        failed_response_after_apply,
        observe=observe,
    )
    assert len(apply_calls) == 1
    assert not requests
    assert bridge.lifecycle.state == loaded.LifecycleState.ATTACHED
    assert bridge.attached is not None


def test_failed_attach_response_rejects_observed_wrong_relative_pose():
    loaded = module()
    bridge, scene, _, _, observe = configured(loaded)

    def failed_response_with_wrong_pose(request):
        scene["world_objects"].clear()
        attached = copy.deepcopy(
            request.scene.robot_state.attached_collision_objects[0]
        )
        attached.object.primitive_poses[0].position.z += 0.01
        scene["attached_objects"].append(attached)
        return False

    with pytest.raises(loaded.PlanningSceneRuntimeError, match="relative pose"):
        bridge.on_status(
            "attached=1;released=0;seq=1",
            loaded.Pose(),
            failed_response_with_wrong_pose,
            observe=observe,
        )
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD
    assert bridge.attached is None


def test_scene_inconsistent_or_absent_fails_closed_on_attached_status():
    loaded = module()
    bridge, scene, _, _, observe = configured(loaded)
    scene["world_objects"].clear()
    with pytest.raises(loaded.PlanningSceneRuntimeError, match="ABSENT"):
        bridge.on_status(
            "attached=1;released=0;seq=1",
            loaded.Pose(),
            lambda _: True,
            observe=observe,
        )

    bridge, scene, _, _, observe = configured(loaded)
    scene["attached_objects"].append(
        loaded.make_attached_fixture_object(
            bridge.collision, bridge.attachment_link, bridge.touch_links, loaded.Pose()
        )
    )
    with pytest.raises(loaded.PlanningSceneRuntimeError, match="INCONSISTENT"):
        bridge.on_status(
            "attached=1;released=0;seq=1",
            loaded.Pose(),
            lambda _: True,
            observe=observe,
        )


def test_attached_scene_metadata_mismatch_fails_closed():
    loaded = module()
    bridge, scene, _, _, observe = configured(loaded)
    attached = loaded.make_attached_fixture_object(
        bridge.collision, bridge.attachment_link, bridge.touch_links, loaded.Pose()
    )
    attached.touch_links = ["unexpected_link"]
    scene["world_objects"].clear()
    scene["attached_objects"].append(attached)
    with pytest.raises(
        loaded.PlanningSceneRuntimeError, match="metadata/link/touch-links"
    ):
        bridge.on_status(
            "attached=1;seq=1", loaded.Pose(), lambda _: True, observe=observe
        )


def test_duplicate_attached_status_is_idempotent_and_does_not_apply_attach_twice():
    loaded = module()
    bridge, scene, requests, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    scene["world_objects"].clear()
    scene["attached_objects"] = [
        loaded.make_attached_fixture_object(
            bridge.collision, bridge.attachment_link, bridge.touch_links, loaded.Pose()
        )
    ]
    # The robot has moved since capture, so the current TF-derived relative pose
    # differs from the canonical attached realization already in the scene.
    moved_tcp_pose = loaded.Pose()
    moved_tcp_pose.position.x = 0.35
    assert bridge.on_status(
        "attached=1;released=0;seq=2", moved_tcp_pose, apply, observe=observe
    )
    assert len(requests) == 1


def test_successful_attach_stores_post_mutation_pose_as_canonical():
    loaded = module()
    bridge, scene, _, apply, observe = configured(loaded)

    def apply_with_scene_pose_normalization(request):
        assert apply(request)
        scene["attached_objects"][0].object.primitive_poses[0].position.x += 2e-6
        return True

    assert bridge.on_status(
        "attached=1;released=0;seq=1",
        loaded.Pose(),
        apply_with_scene_pose_normalization,
        observe=observe,
    )
    assert bridge.attached.object.primitive_poses[0].position.x == pytest.approx(2e-6)

    # Current TF differs from both the request and captured relative pose.
    moved_tcp_pose = loaded.Pose()
    moved_tcp_pose.position.x = 0.35
    assert bridge.on_status(
        "attached=1;released=0;seq=2",
        moved_tcp_pose,
        apply_with_scene_pose_normalization,
        observe=observe,
    )


def test_duplicate_attached_status_rejects_wrong_relative_pose():
    loaded = module()
    bridge, scene, requests, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    scene["world_objects"].clear()
    wrong = copy.deepcopy(bridge.attached)
    wrong.object.primitive_poses[0].position.x += 0.01
    scene["attached_objects"] = [wrong]

    with pytest.raises(loaded.PlanningSceneRuntimeError, match="relative pose"):
        bridge.on_status(
            "attached=1;released=0;seq=2", loaded.Pose(), apply, observe=observe
        )
    assert len(requests) == 1


def test_release_rejects_stale_attached_relative_pose_before_detach():
    loaded = module()
    bridge, scene, requests, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    scene["attached_objects"][0].object.primitive_poses[0].position.y += 0.01

    with pytest.raises(loaded.PlanningSceneRuntimeError, match="relative pose"):
        bridge.on_status(
            "attached=0;released=1;seq=2", None, apply, now=5.0, observe=observe
        )
    assert bridge.lifecycle.state == loaded.LifecycleState.ATTACHED
    assert not bridge._release_observed
    assert len(requests) == 1
    assert scene["attached_objects"]


def test_release_reconciles_attached_scene_even_when_runtime_starts_world():
    loaded = module()
    bridge, scene, requests, apply, observe = configured(loaded)
    scene["world_objects"].clear()
    scene["attached_objects"].append(
        loaded.make_attached_fixture_object(
            bridge.collision, bridge.attachment_link, bridge.touch_links, loaded.Pose()
        )
    )
    assert bridge.on_status(
        "attached=0;released=1;seq=1", None, apply, now=5.0, observe=observe
    )
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD
    assert bridge._release_observed
    assert bridge.attached is None
    assert len(requests) == 1
    assert [x.id for x in scene["world_objects"]] == [bridge.collision.id]
    assert not scene["attached_objects"]


def test_release_reconciles_attached_scene_from_capturing_state():
    loaded = module()
    bridge, scene, requests, apply, observe = configured(loaded)
    bridge.lifecycle.begin_capture()
    scene["world_objects"].clear()
    scene["attached_objects"].append(
        loaded.make_attached_fixture_object(
            bridge.collision, bridge.attachment_link, bridge.touch_links, loaded.Pose()
        )
    )
    assert bridge.on_status("released=1;seq=1", None, apply, now=5.0, observe=observe)
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD
    assert bridge._release_observed
    assert len(requests) == 1


def test_release_with_scene_already_world_reconciles_without_detach_mutation():
    loaded = module()
    bridge, scene, requests, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    scene["attached_objects"].clear()
    scene["world_objects"].append(copy.deepcopy(bridge.collision))
    assert bridge.on_status(
        "released=1;seq=2",
        None,
        apply,
        now=5.0,
        observe=observe,
    )
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD
    assert bridge._release_observed
    assert len(requests) == 1  # attachment only; detach was already in WORLD


def test_duplicate_newer_released_event_does_not_reset_release_time():
    loaded = module()
    bridge, _, requests, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    assert bridge.on_status(
        "released=1;seq=2",
        None,
        apply,
        now=5.0,
        observe=observe,
    )
    bridge.lifecycle.authorize_cleanup(5.0, deadline_at=8.0)
    assert bridge.on_status(
        "released=1;seq=3",
        None,
        lambda request: requests.append(request),
        now=10.0,
        observe=observe,
    )
    assert bridge.lifecycle.cleanup_due(8.0)
    assert len(requests) == 2  # attach and one detach; duplicate release is inert


def test_detach_not_converged_to_world_fails_closed():
    loaded = module()
    bridge, _, _, apply, observe = configured(loaded)
    bridge.on_status("attached=1;seq=1", loaded.Pose(), apply, observe=observe)
    with pytest.raises(
        loaded.PlanningSceneRuntimeError, match="post-release scene=ATTACHED"
    ):
        bridge.on_status(
            "released=1;seq=2", None, lambda _: False, now=5.0, observe=observe
        )
    assert bridge.lifecycle.state == loaded.LifecycleState.ATTACHED
    assert not bridge._release_observed


def test_released_with_absent_scene_fails_closed():
    loaded = module()
    bridge, scene, _, _, observe = configured(loaded)
    scene["world_objects"].clear()
    with pytest.raises(
        loaded.PlanningSceneRuntimeError, match="ABSENT during RELEASED"
    ):
        bridge.on_status(
            "released=1;seq=1", None, lambda _: True, now=5.0, observe=observe
        )
    assert not bridge._release_observed


def test_cleanup_requires_scene_absence_before_accepting_delete_ack():
    loaded = module()
    bridge, scene, _, apply, observe = configured(loaded)
    bridge.on_status("attached=1;seq=1", loaded.Pose(), apply, observe=observe)
    bridge.on_status("released=1;seq=2", None, apply, now=5.0, observe=observe)
    bridge.confirm_place({**confirmation(), "cleanup_deadline_s": 8.0}, now=5.0)
    ack = {
        **confirmation(),
        "status": "PHYSICAL_DELETE_PENDING",
        "cleanup_deadline_s": 8.0,
    }
    with pytest.raises(loaded.PlanningSceneRuntimeError, match="final scene=WORLD"):
        bridge.on_cleanup_timer(8.0, apply, lambda _: None, lambda: ack, observe)
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD


def test_static_scene_profile_excludes_support_overlaps():
    loaded = module()
    assert loaded.MINIMUM_STATIC_OBJECT_IDS == ("pedestal", "cnc_front_top")
    scene = (
        Path(__file__).parents[2]
        / "arm_cell_moveit_config/config/m0609_pnp_static_scene.json"
    )
    assert [
        obj["id"] for obj in __import__("json").loads(scene.read_text())["objects"]
    ] == [
        "pedestal",
        "cnc_front_left",
        "cnc_front_top",
        "camera_rig_post_left",
        "amr_tray_rail_left",
        "amr_tray_rail_right",
        "amr_tray_rail_rear",
    ]


def test_release_without_successful_place_never_becomes_cleanup_eligible():
    loaded = module()
    bridge, _, _, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    bridge.on_status(
        "attached=0;released=1;seq=2", None, apply, now=5.0, observe=observe
    )

    assert (
        bridge.on_cleanup_timer(100.0, apply, lambda _: True, lambda: None, observe)
        is False
    )
    assert (
        bridge.confirm_place({**confirmation(), "target_id": "wrong"}, now=5.0) is False
    )


def test_duplicate_and_older_gripper_sequences_are_ignored():
    loaded = module()
    bridge, _, requests, apply, observe = configured(loaded)

    assert bridge.on_status(
        "attached=1;released=0;seq=3", loaded.Pose(), apply, observe=observe
    )
    assert (
        bridge.on_status("attached=0;released=1;seq=3", None, apply, observe=observe)
        is False
    )
    assert (
        bridge.on_status("attached=0;released=1;seq=2", None, apply, observe=observe)
        is False
    )
    assert bridge.lifecycle.state == loaded.LifecycleState.ATTACHED
    assert len(requests) == 1


def test_failed_or_out_of_volume_place_cannot_authorize_cleanup():
    loaded = module()
    bridge, _, _, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    bridge.on_status(
        "attached=0;released=1;seq=2", None, apply, now=5.0, observe=observe
    )

    assert (
        bridge.confirm_place({**confirmation(), "status": "REJECTED"}, now=5.0) is False
    )
    assert bridge.confirm_place({**confirmation(), "run_id": "wrong"}, now=5.0) is False
    assert (
        bridge.on_cleanup_timer(100.0, apply, lambda _: True, lambda: None, observe)
        is False
    )


def test_remove_failure_does_not_issue_delete_request_or_complete_cleanup():
    loaded = module()
    bridge, scene, _, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    bridge.on_status(
        "attached=0;released=1;seq=2", None, apply, now=5.0, observe=observe
    )
    bridge.confirm_place({**confirmation(), "cleanup_deadline_s": 8.0}, now=5.0)
    requests = []

    with pytest.raises(loaded.PlanningSceneRuntimeError):
        bridge.on_cleanup_timer(
            8.0, lambda _: False, requests.append, lambda: None, observe
        )

    assert requests == []
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD


def test_stale_ack_does_not_claim_cleanup_complete_and_retry_is_idempotent():
    loaded = module()
    bridge, scene, _, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    bridge.on_status(
        "attached=0;released=1;seq=2", None, apply, now=5.0, observe=observe
    )
    bridge.confirm_place({**confirmation(), "cleanup_deadline_s": 8.0}, now=5.0)
    requests = []
    stale_ack = {**confirmation(), "decision_id": "foreign", "status": "DELETED"}

    assert (
        bridge.on_cleanup_timer(8.0, apply, requests.append, lambda: stale_ack, observe)
        is False
    )
    assert (
        bridge.on_cleanup_timer(8.0, apply, requests.append, lambda: stale_ack, observe)
        is False
    )
    assert bridge.lifecycle.state == loaded.LifecycleState.WORLD
    assert len(requests) == 2


def test_stale_confirmation_is_not_authorized():
    loaded = module()
    bridge, _, _, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    bridge.on_status(
        "attached=0;released=1;seq=2", None, apply, now=5.0, observe=observe
    )

    assert (
        bridge.confirm_place({**confirmation(), "cleanup_deadline_s": 4.0}, now=5.0)
        is False
    )


def test_cleanup_confirmation_does_not_require_fixture_position_observation():
    loaded = module()
    bridge, _, _, apply, observe = configured(loaded)
    bridge.on_status(
        "attached=1;released=0;seq=1", loaded.Pose(), apply, observe=observe
    )
    bridge.on_status(
        "attached=0;released=1;seq=2", None, apply, now=5.0, observe=observe
    )
    payload = confirmation()
    payload.pop("pose")
    payload["post_release_position_observation"] = "UNAVAILABLE"

    assert bridge.confirm_place(payload, now=5.0)


def test_released_world_without_prior_attached_state_is_ignored():
    loaded = module()
    bridge, _, _, apply, observe = configured(loaded)

    assert not bridge.on_status(
        "attached=0;released=1;seq=1",
        None,
        apply,
        now=1.0,
        observe=observe,
    )

    assert not bridge._release_observed


def test_place_phase_alone_does_not_write_success_event(tmp_path):
    loaded = module()
    event_path = tmp_path / "place_result_event.json"
    producer = loaded.PlaceResultEventProducer("run-1", "cube_profile", event_path)

    assert not producer.observe_feedback("goal-1", 4, now=10.0)
    assert not event_path.exists()
    assert producer.observe_release(
        now=11.0, fresh_released=True, detach_confirmed=True
    )
    event = __import__("json").loads(event_path.read_text())
    assert event["schema_version"] == "wu14-pnp-place-result/v1"
    assert event["status"] == "SUCCESS"
    assert event["run_id"] == "run-1"
    assert event["target_id"] == "cube_profile"
    assert event["result_identity"] == "goal-1:1"
    assert event["fresh_released"] is True
    assert event["detach_confirmed"] is True
    assert event["release_motion_completed"] is True
    assert not producer.observe_feedback("goal-1", 4, now=12.0)
    assert event_path.read_text() == event_path.read_text()


def test_detach_fact_emits_before_returning_home_feedback(tmp_path):
    loaded = module()
    event_path = tmp_path / "place_result_event.json"
    producer = loaded.PlaceResultEventProducer("run-1", "cube_profile", event_path)
    producer.observe_feedback("goal-1", loaded.MISSION_PHASE_PLACING, now=1.0)

    assert producer.observe_release(
        now=1.01, fresh_released=True, detach_confirmed=True
    )
    assert event_path.exists()
    event = json.loads(event_path.read_text())
    assert event["timestamp"] == 1.01


def test_failed_cancelled_or_foreign_feedback_does_not_write_place_event(tmp_path):
    loaded = module()
    event_path = tmp_path / "place_result_event.json"
    producer = loaded.PlaceResultEventProducer("run-1", "cube_profile", event_path)

    assert not producer.observe_feedback("goal-foreign", 4, now=1.0)
    assert not producer.observe_feedback("goal-foreign", 7, now=2.0)
    assert not producer.observe_feedback("goal-foreign", 5, now=3.0)
    assert not event_path.exists()


def test_place_release_fact_outside_placing_phase_does_not_emit(tmp_path):
    loaded = module()
    event_path = tmp_path / "place_result_event.json"
    producer = loaded.PlaceResultEventProducer("run-1", "cube_profile", event_path)

    assert not producer.observe_feedback(
        "goal-1",
        loaded.MISSION_PHASE_RETURNING_HOME,
        now=1.0,
        fresh_released=True,
        detach_confirmed=True,
    )
    assert not event_path.exists()


@pytest.mark.parametrize(
    "facts", [{"detach_confirmed": True}, {"fresh_released": True}]
)
def test_release_requires_fresh_release_and_detach_facts(tmp_path, facts):
    loaded = module()
    event_path = tmp_path / "place_result_event.json"
    producer = loaded.PlaceResultEventProducer("run-1", "cube_profile", event_path)

    assert not producer.observe_feedback(
        "goal-1", loaded.MISSION_PHASE_PLACING, now=1.0, **facts
    )
    assert not event_path.exists()
