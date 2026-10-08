import math
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

import pnp_validation.fixture as fixture_module
from pnp_validation.fixture import (
    Aabb,
    FixtureOwnership,
    FixtureProfile,
    Pose,
    PoseReference,
    ValidationManifest,
    cleanup_decision,
    sample_spawn_pose,
    top_surface_reference,
)
from pnp_validation.run_validation import (
    CleanupRequestWorker,
    MissionCompletionEventWorker,
    PlaceResultEventWorker,
    IsaacFixtureRuntime,
    ValidationRun,
    capture_config_from_profile,
    capture_reference_config_from_profile,
    fixture_profile_from_validation_config,
    load_validation_profile,
    validate_validation_profile,
)
from pnp_validation.run_validation import _angular_distance


def profile(**overrides):
    values = dict(
        target_id="cube_profile",
        fixture_dimensions=(0.14, 0.14, 0.08),
        spawn_bounds=Aabb((0.15, -0.35, 0.72), (0.55, 0.25, 0.95)),
        excluded_volumes=(Aabb((0.25, -0.05, 0.70), (0.40, 0.10, 0.90)),),
        seed=17,
        expected_fixture_release_position_base=(0.5, 0.25, 0.3675),
        place_position_tolerance_m=0.05,
        registration_ttl_s=30.0,
        deletion_delay_s=3.0,
    )
    values.update(overrides)
    return FixtureProfile(**values)


def test_validation_fixture_uses_selected_json_recipe_from_profile_registry():
    project_root = Path(__file__).resolve().parents[6]
    config_path = (
        project_root
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )

    config = load_validation_profile(config_path)
    loaded = fixture_profile_from_validation_config(config)

    assert loaded.target_id == "cube_profile"
    assert loaded.expected_fixture_release_position_base == pytest.approx(
        (0.5, 0.25, 0.4075)
    )


def test_same_seed_produces_same_valid_pose():
    first = sample_spawn_pose(profile())
    second = sample_spawn_pose(profile())

    assert first == second
    assert profile().spawn_bounds.contains_center(first, profile().fixture_dimensions)
    assert not any(
        excluded.intersects_center(first, profile().fixture_dimensions)
        for excluded in profile().excluded_volumes
    )


def test_fixture_profile_loader_is_shared_by_live_entrypoints():
    config = {
        "validation": {
            "target_id": "cube_profile",
            "registration_ttl_s": 30.0,
            "registration_ttl_max_s": 60.0,
        },
        "fixture": {
            "dimensions_m": [0.07, 0.07, 0.08],
            "spawn_bounds_m": {"min": [0.0, 0.0, 0.7], "max": [1.0, 1.0, 1.0]},
            "excluded_volumes_m": [],
            "seed": 17,
            "geometry_frame": "world",
            "support_surface_id": "/World/SF_Twin_Cell/AMR/Mockup/Tray",
            "support_surface_frame": "amr_top",
            "spawn_clearance_m": 0.001,
        },
        "place": {
            "position_tolerance_m": 0.05,
            "deletion_delay_s": 3.0,
        },
        "orchestration": {
            "mission_target_id": "cube_profile",
        },
        "_mission_recipe": {
            "target_id": "cube_profile",
            "place": {
                "desired_object_pose": {
                    "position": [0.5, 0.25, 0.37],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
            },
        },
        "motion": {
            "observation_to_object_translation": [0.0, 0.0, -0.04],
            "object_to_grasp_tcp_translation": [0.0, 0.0, 0.0375],
            "object_to_grasp_tcp_quaternion": [1.0, 0.0, 0.0, 0.0],
        },
        "capture": {"orientation_tolerance_deg": 10.0},
    }

    loaded = fixture_profile_from_validation_config(config)

    assert loaded.target_id == "cube_profile"
    assert loaded.fixture_dimensions == (0.07, 0.07, 0.08)
    assert loaded.support_surface_id == "/World/SF_Twin_Cell/AMR/Mockup/Tray"
    assert loaded.expected_fixture_release_position_base == pytest.approx(
        (0.5, 0.25, 0.4075)
    )


def test_different_seed_produces_different_deterministic_valid_pose():
    first = sample_spawn_pose(profile(seed=17))
    second = sample_spawn_pose(profile(seed=18))

    assert first != second
    assert sample_spawn_pose(profile(seed=18)) == second


def test_excluded_volume_is_rejected_and_resampled():
    bounds = Aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    excluded = Aabb((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))

    with pytest.raises(ValueError, match="unable to sample"):
        sample_spawn_pose(
            profile(
                spawn_bounds=bounds,
                excluded_volumes=(excluded,),
                fixture_dimensions=(0.1, 0.1, 0.1),
            )
        )


def test_top_surface_reference_uses_visible_centroid_in_base_link():
    pose = Pose(0.30, -0.10, 0.80, yaw=math.pi / 2)

    reference = top_surface_reference(pose, (0.14, 0.20, 0.08))

    assert reference.frame_id == "base_link"
    assert reference.x == pytest.approx(0.30)
    assert reference.y == pytest.approx(-0.10)
    assert reference.z == pytest.approx(0.84)
    assert reference.yaw == pytest.approx(math.pi / 2)


def test_expected_fixture_center_applies_inverse_object_to_tcp_transform():
    derive = getattr(fixture_module, "fixture_center_from_tcp_target", None)
    assert callable(derive)
    expected = derive(
        tcp_position=(0.50, 0.25, 0.37),
        tcp_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        object_to_tcp_translation=(0.0, 0.0, -0.0025),
        object_to_tcp_quaternion_xyzw=(1.0, 0.0, 0.0, 0.0),
    )

    assert expected == pytest.approx((0.50, 0.25, 0.3675))


def test_cleanup_keeps_fixture_tolerance_as_post_release_observation():
    ownership = FixtureOwnership({"/World/Validation/Cube"})
    expected = (0.65, 0.10, 1.0475)
    # x=0.69 is beyond the former place.volume_m maximum (0.65); target-relative
    # acceptance is now authoritative.
    actual = Pose(0.69, 0.10, 1.0475)

    decision = cleanup_decision(
        ownership,
        prim_path="/World/Validation/Cube",
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=actual,
        expected_position=expected,
        position_tolerance_m=0.05,
    )

    assert decision.eligible
    assert decision.position_error_m == pytest.approx(0.04)
    assert decision.expected_position == expected
    assert decision.actual_position == (actual.x, actual.y, actual.z)

    outside = cleanup_decision(
        ownership,
        prim_path="/World/Validation/Cube",
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.7001, 0.10, 1.0475),
        expected_position=expected,
        position_tolerance_m=0.05,
    )
    assert outside.eligible
    assert outside.post_release_position_observation == "OUTSIDE_TOLERANCE"
    assert outside.position_error_m == pytest.approx(0.0501)


@pytest.mark.parametrize(
    "overrides,reason",
    [
        ({"place_succeeded": False}, "PLACE did not succeed"),
        ({"fresh_released": False}, "fresh RELEASED was not confirmed"),
        ({"detach_confirmed": False}, "fixture is not confirmed detached"),
        ({"ownership_unambiguous": False}, "fixture ownership is ambiguous"),
    ],
)
def test_cleanup_requires_all_live_acceptance_gates(overrides, reason):
    values = dict(
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.50, 0.25, 0.3675),
    )
    values.update(overrides)
    decision = cleanup_decision(
        FixtureOwnership({"/World/Validation/Cube"}),
        prim_path="/World/Validation/Cube",
        expected_position=(0.50, 0.25, 0.3675),
        position_tolerance_m=0.05,
        **values,
    )

    assert not decision.eligible
    assert decision.reason == reason


def test_fixture_ownership_never_deletes_foreign_or_attached_prim():
    ownership = FixtureOwnership()
    ownership.claim("/World/Validation/Cube")

    assert cleanup_decision(
        ownership,
        prim_path="/World/Validation/Cube",
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position=(0.5, 0.25, 0.3675),
        position_tolerance_m=0.05,
    ).eligible
    assert not cleanup_decision(
        ownership,
        prim_path="/World/Foreign/Cube",
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position=(0.5, 0.25, 0.3675),
        position_tolerance_m=0.05,
    ).eligible
    assert not cleanup_decision(
        ownership,
        prim_path="/World/Validation/Cube",
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=False,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position=(0.5, 0.25, 0.3675),
        position_tolerance_m=0.05,
    ).eligible


def test_place_outside_target_tolerance_does_not_block_cleanup():
    ownership = FixtureOwnership({"/World/Validation/Cube"})

    decision = cleanup_decision(
        ownership,
        prim_path="/World/Validation/Cube",
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(1.0, 1.0, 1.0),
        expected_position=(0.5, 0.25, 0.3675),
        position_tolerance_m=0.05,
    )

    assert decision.eligible
    assert decision.post_release_position_observation == "OUTSIDE_TOLERANCE"


def test_missing_post_release_pose_is_diagnostic_only_for_cleanup():
    decision = cleanup_decision(
        FixtureOwnership({"/World/Validation/Cube"}),
        prim_path="/World/Validation/Cube",
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=None,
        expected_position=(0.5, 0.25, 0.3675),
        position_tolerance_m=0.05,
    )

    assert decision.eligible
    assert decision.post_release_position_observation == "UNAVAILABLE"


def test_successful_detached_place_schedules_at_three_seconds():
    ownership = FixtureOwnership({"/World/Validation/Cube"})

    decision = cleanup_decision(
        ownership,
        prim_path="/World/Validation/Cube",
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position=(0.5, 0.25, 0.3675),
        position_tolerance_m=0.05,
        now_s=10.0,
        delay_s=3.0,
    )

    assert decision.deadline_s == pytest.approx(13.0)


def test_cleanup_scheduler_is_not_due_before_deadline_and_is_due_at_deadline():
    ownership = FixtureOwnership({"/World/Validation/Cube"})
    decision = cleanup_decision(
        ownership,
        prim_path="/World/Validation/Cube",
        place_succeeded=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position=(0.5, 0.25, 0.3675),
        position_tolerance_m=0.05,
        now_s=10.0,
        delay_s=3.0,
    )

    assert not decision.is_due(12.99)
    assert decision.is_due(13.0)


def test_abort_cleanup_deletes_only_owned_prims():
    ownership = FixtureOwnership({"/World/Validation/Cube"})

    assert ownership.abort_cleanup(
        ["/World/Validation/Cube", "/World/Foreign/Cube"]
    ) == ["/World/Validation/Cube"]


def test_manifest_contains_registered_top_surface_and_run_context():
    fixture_profile = profile()
    pose = Pose(0.2, 0.0, 0.8, yaw=0.1)
    manifest = ValidationManifest.from_fixture(
        run_id="run-1",
        profile=fixture_profile,
        prim_path="/World/Validation/Cube",
        spawn_pose_world=pose,
        registered_top_surface_pose_base_link=PoseReference(
            0.2, 0.0, 0.84, 0.1, "base_link"
        ),
    )

    data = manifest.to_dict()
    assert data["run_id"] == "run-1"
    assert data["target_id"] == "cube_profile"
    assert data["fixture_prim_identity"] == "/World/Validation/Cube"
    assert data["spawn_pose_world"]["frame_id"] == "world"
    assert data["registered_top_surface_pose_base_link"]["frame_id"] == "base_link"
    assert data["registered_top_surface_pose_base_link"]["yaw"] == pytest.approx(0.1)
    assert data["ttl_s"] == 30.0


def test_validation_profile_declares_world_geometry_and_place_tolerance():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())

    assert profile_data["fixture"]["geometry_frame"] == "world"
    assert profile_data["place"]["position_tolerance_m"] == pytest.approx(0.05)
    assert validate_validation_profile(profile_data) is profile_data
    assert profile_data["fixture"]["support_surface_id"] == (
        "/World/SF_Twin_Cell/AMR/Mockup/Tray"
    )
    assert profile_data["fixture"]["support_surface_frame"] == "amr_top"


def test_validation_profile_rejects_missing_geometry_frame():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())
    del profile_data["fixture"]["geometry_frame"]

    with pytest.raises(ValueError, match="geometry_frame"):
        validate_validation_profile(profile_data)


def test_validation_profile_rejects_missing_or_invalid_place_tolerance():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())
    del profile_data["place"]["position_tolerance_m"]

    with pytest.raises(ValueError, match="position_tolerance_m"):
        validate_validation_profile(profile_data)

    profile_data = yaml.safe_load(path.read_text())
    profile_data["place"]["position_tolerance_m"] = 0.0
    with pytest.raises(ValueError, match="position_tolerance_m"):
        validate_validation_profile(profile_data)

    profile_data = yaml.safe_load(path.read_text())
    profile_data["fixture"]["geometry_frame"] = "map"
    with pytest.raises(ValueError, match="geometry_frame"):
        validate_validation_profile(profile_data)


def test_validation_profile_rejects_grasp_offset_drift_between_motion_and_recipe():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())
    profile_data["motion"]["object_to_grasp_tcp_translation"][2] = 0.0

    with pytest.raises(ValueError, match="offset"):
        validate_validation_profile(profile_data)


def test_validation_profile_rejects_nonzero_grasp_translation_xy():
    path = (
        Path(__file__).parents[6]
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
    profile_data = yaml.safe_load(path.read_text())
    profile_data["motion"]["object_to_grasp_tcp_translation"][0] = 0.001

    with pytest.raises(ValueError, match="x must be zero"):
        validate_validation_profile(profile_data)


class RuntimeSpy:
    def __init__(self):
        self.existing = {"/World/Validation/Cube", "/World/Foreign/Cube"}
        self.deleted = []
        self.registration = None
        self.registry = set()
        self.gravity_enabled = []
        self.gravity_enable_attempts = []
        self.fail_gravity_enable = False
        self.track_pose_order = False
        self.events = []

    def spawn_cube(self, prim_path, pose, dimensions):
        self.spawned = (prim_path, pose, dimensions)

    def disable_gravity(self, prim_path):
        self.gravity_disabled = prim_path
        self.events.append(("gravity_off", prim_path))

    def enable_gravity(self, prim_path):
        self.gravity_enable_attempts.append(prim_path)
        if self.fail_gravity_enable:
            raise RuntimeError("test gravity activation failure")
        self.gravity_enabled.append(prim_path)
        self.events.append(("gravity_on", prim_path))

    def register_eligible(self, prim_path, run_id):
        self.eligible = (prim_path, run_id)
        self.registry.add(prim_path)

    def read_world_pose(self, prim_path):
        if self.track_pose_order:
            self.events.append(("read_world_pose", prim_path))
        return getattr(self, "world_pose_override", self.spawned[1])

    def read_base_pose(self, prim_path):
        return self.spawned[1]

    def existing_prim_paths(self):
        return list(self.existing)

    def delete_prim(self, prim_path):
        self.deleted.append(prim_path)

    def retire_fixture_for_pending_delete(self, prim_path):
        self.retired = prim_path
        self.registry.discard(prim_path)
        self.disable_gravity(prim_path)
        return True


def test_validation_run_propagates_target_and_run_context():
    runtime = RuntimeSpy()
    run = ValidationRun(profile=profile(), runtime=runtime, run_id="run-42")

    manifest = run.start()

    assert runtime.eligible == (manifest.fixture_prim_identity, "run-42")
    assert manifest.run_id == "run-42"
    assert manifest.target_id == "cube_profile"
    assert manifest.ttl_s == 30.0
    assert manifest.registered_top_surface_pose.frame_id == "base_link"


def test_validation_run_keeps_gravity_off_until_successful_place_eligibility():
    runtime = RuntimeSpy()
    run = ValidationRun(profile=profile(), runtime=runtime, run_id="run-42")
    runtime.existing.add(run.prim_path)
    run.start()

    assert runtime.gravity_disabled == run.prim_path
    assert runtime.gravity_enabled == []

    run.on_place_result(
        success=False,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
    )

    assert runtime.gravity_enabled == []


@pytest.mark.parametrize(
    "overrides",
    [
        {"fresh_released": False},
        {"detach_confirmed": False},
        {"ownership_unambiguous": False},
    ],
)
def test_validation_run_does_not_enable_gravity_without_all_release_gates(overrides):
    runtime = RuntimeSpy()
    run = ValidationRun(profile=profile(), runtime=runtime, run_id="run-42")
    runtime.existing.add(run.prim_path)
    run.start()
    values = dict(
        success=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
    )
    values.update(overrides)

    decision = run.on_place_result(**values)

    assert not decision.eligible
    assert runtime.gravity_enabled == []


def test_validation_run_enables_gravity_once_after_successful_place_eligibility():
    runtime = RuntimeSpy()
    run = ValidationRun(profile=profile(), runtime=runtime, run_id="run-42")
    runtime.existing.add(run.prim_path)
    run.start()
    values = dict(
        success=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
    )

    first = run.on_place_result(**values)
    second = run.on_place_result(**values)

    assert first.eligible
    assert second.eligible
    assert runtime.gravity_enabled == [run.prim_path]
    assert [event for event in runtime.events if event[0].startswith("gravity_")] == [
        ("gravity_off", run.prim_path),
        ("gravity_on", run.prim_path),
    ]


def test_gravity_activation_failure_preserves_place_acceptance_and_cleanup(tmp_path):
    runtime = RuntimeSpy()
    runtime.fail_gravity_enable = True
    confirmation_path = tmp_path / "place-confirmation.json"
    run = ValidationRun(
        profile=profile(),
        runtime=runtime,
        run_id="run-42",
        clock_s=lambda: 10.0,
        cleanup_confirmation_path=confirmation_path,
    )
    runtime.existing.add(run.prim_path)
    run.start()

    decision = run.on_place_result(
        success=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
    )

    assert decision.eligible
    assert run.cleanup.eligible
    assert runtime.gravity_enable_attempts == [run.prim_path]
    assert (
        run.process_delete_request(
            {
                "schema_version": "wu14-pnp-cleanup/v1",
                "status": "DELETE_REQUESTED",
                "run_id": "run-42",
                "target_id": "cube_profile",
                "fixture_prim_identity": run.prim_path,
                "decision_id": run.cleanup_decision_id,
                "fresh_released": True,
                "detach_confirmed": True,
                "ownership_unambiguous": True,
                "scene_absent_confirmed": True,
            },
            tmp_path / "cleanup-ack.json",
            now_s=13.0,
        )
        is True
    )
    assert runtime.retired == run.prim_path
    assert [event for event in runtime.events if event[0].startswith("gravity_")] == [
        ("gravity_off", run.prim_path),
        ("gravity_off", run.prim_path),
    ]


def test_validation_run_abort_preserves_foreign_state():
    runtime = RuntimeSpy()
    run = ValidationRun(profile=profile(), runtime=runtime, run_id="run-42")
    runtime.existing.add(run.prim_path)
    run.start()

    run.abort()

    assert runtime.deleted == [run.prim_path]


def test_validation_run_fails_closed_on_world_yaw_readback_mismatch():
    runtime = RuntimeSpy()
    run = ValidationRun(profile=profile(), runtime=runtime, run_id="run-42")
    runtime.existing.add(run.prim_path)
    runtime.world_pose_override = Pose(0.2, 0.0, 0.8, yaw=1.0)

    with pytest.raises(RuntimeError, match="world yaw readback"):
        run.start()

    assert runtime.deleted == [run.prim_path]


def test_cleanup_request_defers_physical_delete_until_mission_completion(tmp_path):
    runtime = RuntimeSpy()
    run = ValidationRun(
        profile=profile(),
        runtime=runtime,
        run_id="run-42",
        clock_s=lambda: 10.0,
        cleanup_confirmation_path=tmp_path / "confirmation.json",
    )
    runtime.existing.add(run.prim_path)
    run.start()
    decision = run.on_place_result(
        success=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
    )
    request = {
        "schema_version": "wu14-pnp-cleanup/v1",
        "status": "DELETE_REQUESTED",
        "run_id": "run-42",
        "target_id": "cube_profile",
        "fixture_prim_identity": run.prim_path,
        "decision_id": run.cleanup_decision_id,
        "fresh_released": True,
        "detach_confirmed": True,
        "ownership_unambiguous": True,
        "scene_absent_confirmed": True,
    }
    ack_path = tmp_path / "cleanup_ack.json"

    assert run.process_delete_request(request, ack_path, now_s=12.99) is False
    assert run.process_delete_request(request, ack_path, now_s=decision.deadline_s)
    assert runtime.deleted == []
    assert runtime.retired == run.prim_path
    assert run.prim_path not in runtime.registry
    assert runtime.gravity_disabled == run.prim_path
    assert [event for event in runtime.events if event[0].startswith("gravity_")] == [
        ("gravity_off", run.prim_path),
        ("gravity_on", run.prim_path),
        ("gravity_off", run.prim_path),
    ]
    assert run.physical_delete_state == "pending_physical_delete"
    assert json.loads(ack_path.read_text())["status"] == "PHYSICAL_DELETE_PENDING"

    completion = {
        "schema_version": "wu14-pnp-mission-completion/v1",
        "status": "DEPLETED",
        "run_id": "run-42",
        "target_id": "cube_profile",
        "goal_id": "goal-42",
        "go_home_complete": True,
        "scene_absent": True,
        "action_status": 4,
        "exit_reason": "DEPLETED",
    }
    assert run.complete_mission(completion, ack_path)
    assert runtime.deleted == [run.prim_path]
    assert run.physical_delete_state == "physically_deleted"
    assert json.loads(ack_path.read_text())["status"] == "DELETED"
    assert run.complete_mission(completion, ack_path)
    assert runtime.deleted == [run.prim_path]


def test_successful_place_writes_validated_cleanup_confirmation_atomically(tmp_path):
    runtime = RuntimeSpy()
    runtime.track_pose_order = True
    run = ValidationRun(profile=profile(), runtime=runtime, run_id="run-42")
    runtime.existing.add(run.prim_path)
    run.start()
    runtime.world_pose_override = Pose(0.5, 0.25, 0.3675)
    confirmation_path = tmp_path / "place_confirmation.json"

    decision = run.on_place_result(
        success=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=None,
        expected_position_world=(0.5, 0.25, 0.3675),
        confirmation_path=confirmation_path,
    )

    payload = json.loads(confirmation_path.read_text())
    assert decision.eligible
    assert payload["run_id"] == "run-42"
    assert payload["target_id"] == "cube_profile"
    assert payload["fixture_prim_identity"] == run.prim_path
    assert payload["decision_id"]
    assert payload["status"] == "AUTHORIZED"
    assert payload["post_release_position_observation"] == "UNAVAILABLE"
    assert run.post_release_observation_pending
    assert run.observe_post_release_position() is None
    assert run.observe_post_release_position() == "PASS"
    assert runtime.events.index(("gravity_on", run.prim_path)) < max(
        index
        for index, event in enumerate(runtime.events)
        if event == ("read_world_pose", run.prim_path)
    )


def test_failed_or_unconfirmed_place_does_not_write_cleanup_confirmation(tmp_path):
    runtime = RuntimeSpy()
    run = ValidationRun(profile=profile(), runtime=runtime, run_id="run-42")
    runtime.existing.add(run.prim_path)
    run.start()
    confirmation_path = tmp_path / "place_confirmation.json"

    run.on_place_result(
        success=False,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
        confirmation_path=confirmation_path,
    )

    assert not confirmation_path.exists()


def test_outside_place_target_tolerance_is_recorded_without_blocking_cleanup(tmp_path):
    runtime = RuntimeSpy()
    run = ValidationRun(profile=profile(), runtime=runtime, run_id="run-42")
    runtime.existing.add(run.prim_path)
    run.start()
    confirmation_path = tmp_path / "place_confirmation.json"

    decision = run.on_place_result(
        success=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.56, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
        confirmation_path=confirmation_path,
    )

    assert decision.eligible
    assert decision.post_release_position_observation == "OUTSIDE_TOLERANCE"
    assert confirmation_path.exists()


def test_delete_request_requires_matching_authorization_and_ack_is_idempotent(
    tmp_path,
):
    runtime = RuntimeSpy()
    run = ValidationRun(
        profile=profile(), runtime=runtime, run_id="run-42", clock_s=lambda: 10.0
    )
    runtime.existing.add(run.prim_path)
    run.start()
    confirmation_path = tmp_path / "place_confirmation.json"
    decision = run.on_place_result(
        success=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
        confirmation_path=confirmation_path,
    )
    request = json.loads(confirmation_path.read_text())
    ack_path = tmp_path / "delete_ack.json"

    assert not run.process_delete_request(
        {**request, "decision_id": "wrong"}, ack_path, now_s=decision.deadline_s
    )
    request["status"] = "DELETE_REQUESTED"
    request.update(
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        scene_absent_confirmed=True,
    )
    assert run.process_delete_request(request, ack_path, now_s=decision.deadline_s)
    assert run.process_delete_request(request, ack_path, now_s=decision.deadline_s)
    assert json.loads(ack_path.read_text())["status"] == "PHYSICAL_DELETE_PENDING"
    assert runtime.deleted == []


def test_cleanup_request_worker_retries_pending_request_and_ignores_foreign_context(
    tmp_path,
):
    runtime = RuntimeSpy()
    run = ValidationRun(
        profile=profile(), runtime=runtime, run_id="run-42", clock_s=lambda: 10.0
    )
    runtime.existing.add(run.prim_path)
    run.start()
    confirmation_path = tmp_path / "place_confirmation.json"
    decision = run.on_place_result(
        success=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
        confirmation_path=confirmation_path,
    )
    request_path = tmp_path / "delete_request.json"
    ack_path = tmp_path / "delete_ack.json"
    request = json.loads(confirmation_path.read_text())
    request["status"] = "DELETE_REQUESTED"
    request.update(
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        scene_absent_confirmed=True,
    )
    request_path.write_text(json.dumps(request))
    worker = CleanupRequestWorker(run, request_path, ack_path)

    assert not worker.poll(now_s=decision.deadline_s - 0.01)
    assert worker.poll(now_s=decision.deadline_s)
    assert worker.poll(now_s=decision.deadline_s)
    assert json.loads(ack_path.read_text())["status"] == "PHYSICAL_DELETE_PENDING"
    assert runtime.deleted == []


def test_mission_completion_worker_deletes_pending_fixture_once(tmp_path):
    runtime = RuntimeSpy()
    run = ValidationRun(
        profile=profile(),
        runtime=runtime,
        run_id="run-42",
        cleanup_confirmation_path=tmp_path / "confirmation.json",
    )
    runtime.existing.add(run.prim_path)
    run.start()
    decision = run.on_place_result(
        success=True,
        fresh_released=True,
        detach_confirmed=True,
        ownership_unambiguous=True,
        actual_pose=Pose(0.5, 0.25, 0.3675),
        expected_position_world=(0.5, 0.25, 0.3675),
    )
    ack_path = tmp_path / "delete_ack.json"
    request = {
        "schema_version": "wu14-pnp-cleanup/v1",
        "status": "DELETE_REQUESTED",
        "run_id": "run-42",
        "target_id": "cube_profile",
        "fixture_prim_identity": run.prim_path,
        "decision_id": run.cleanup_decision_id,
        "fresh_released": True,
        "detach_confirmed": True,
        "ownership_unambiguous": True,
        "scene_absent_confirmed": True,
    }
    assert run.process_delete_request(request, ack_path, now_s=decision.deadline_s)
    event_path = tmp_path / "mission_complete.json"
    event = {
        "schema_version": "wu14-pnp-mission-completion/v1",
        "status": "DEPLETED",
        "run_id": "run-42",
        "target_id": "cube_profile",
        "goal_id": "goal-42",
        "go_home_complete": False,
        "scene_absent": True,
        "action_status": 4,
        "exit_reason": "DEPLETED",
    }
    event_path.write_text(json.dumps(event))
    worker = MissionCompletionEventWorker(run, event_path, ack_path)

    assert not worker.poll()
    assert runtime.deleted == []
    event["go_home_complete"] = True
    event_path.write_text(json.dumps(event))
    assert worker.poll()
    assert worker.poll()
    assert runtime.deleted == [run.prim_path]


def test_place_result_event_worker_calls_record_once_and_persists_consumption(
    tmp_path,
):
    event_path = tmp_path / "place_result_event.json"
    consumed_path = tmp_path / "place_result_event.consumed.json"
    event_path.write_text(
        json.dumps(
            {
                "schema_version": "wu14-pnp-place-result/v1",
                "status": "SUCCESS",
                "run_id": "run-42",
                "target_id": "cube_profile",
                "result_identity": "goal-1:1",
                "sequence": 1,
            }
        )
    )
    calls = []
    worker = PlaceResultEventWorker(
        "run-42",
        "cube_profile",
        event_path,
        consumed_path,
        lambda event: calls.append(event) or True,
    )

    assert worker.poll()
    assert not worker.poll()
    restarted = PlaceResultEventWorker(
        "run-42",
        "cube_profile",
        event_path,
        consumed_path,
        lambda event: calls.append(event) or True,
    )
    assert not restarted.poll()
    assert len(calls) == 1


def test_capture_profile_maps_to_task3_capture_config_in_degrees():
    config = capture_config_from_profile(
        {
            "capture": {
                "volume_dimensions_m": [0.12, 0.12, 0.12],
                "position_tolerance_m": 0.015,
                "orientation_tolerance_deg": 10.0,
                "expected_contact_width_mm": 70.0,
                "contact_width_tolerance_mm": 2.0,
            }
        }
    )

    assert config.capture_volume_dimensions_m == (0.12, 0.12, 0.12)
    assert config.orientation_tolerance_deg == 10.0
    assert config.validate() is config


def test_capture_reference_profile_maps_fixture_geometry_and_motion_semantics():
    config = capture_reference_config_from_profile(
        {
            "fixture": {"dimensions_m": [0.07, 0.07, 0.08]},
            "motion": {
                "observation_reference": "fixture_profile_reference",
                "observation_to_object_translation": [0.0, 0.0, -0.04],
                "object_to_grasp_tcp_translation": [0.0, 0.0, 0.0375],
                "insertion_axis_tcp": [0.0, 0.0, 1.0],
            },
        }
    )

    assert config.fixture_dimensions_m == (0.07, 0.07, 0.08)
    assert config.observation_to_grasp_tcp_m == pytest.approx(-0.0025)
    assert config.insertion_axis_tcp == (0.0, 0.0, 1.0)


def test_base_link_world_transform_round_trip_is_explicit():
    transform = IsaacFixtureRuntime.base_link_world_transform(
        world_to_base=(1.0, -2.0, 0.5, math.pi / 2.0)
    )
    base_pose = Pose(0.2, 0.3, 0.4, yaw=0.25)

    world_pose = transform.base_to_world(base_pose)

    assert transform.world_to_base(world_pose).x == pytest.approx(base_pose.x)
    assert transform.world_to_base(world_pose).y == pytest.approx(base_pose.y)
    assert transform.world_to_base(world_pose).z == pytest.approx(base_pose.z)
    assert transform.world_to_base(world_pose).yaw == pytest.approx(base_pose.yaw)


class _Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.xyz = (x, y, z)

    def GetReal(self):
        return self.w

    def GetImaginary(self):
        return self.xyz


def test_yaw_extraction_preserves_pure_z_rotation():
    yaw = -0.10956567444761856
    quaternion = _Quaternion(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))

    assert IsaacFixtureRuntime._yaw_from_quaternion(quaternion) == pytest.approx(yaw)


def test_yaw_extraction_handles_nonzero_x_and_y_components():
    roll, pitch, yaw = 0.31, -0.27, -0.10956567444761856
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    quaternion = _Quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )

    assert quaternion.xyz[0] != 0.0
    assert quaternion.xyz[1] != 0.0
    assert IsaacFixtureRuntime._yaw_from_quaternion(quaternion) == pytest.approx(yaw)

    extracted_roll, extracted_pitch, extracted_yaw = (
        IsaacFixtureRuntime._rpy_from_quaternion(quaternion)
    )
    assert extracted_roll == pytest.approx(roll)
    assert extracted_pitch == pytest.approx(pitch)
    assert extracted_yaw == pytest.approx(yaw)


def _scaled_rotation_matrix(quaternion, scales):
    w, x, y, z = quaternion
    rotation = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )
    # USD/Gf matrix convention used by the runtime is the transpose of the
    # row-vector quaternion convention used by the pure conversion helper.
    usd_rotation = tuple(
        tuple(rotation[column][row] for column in range(3)) for row in range(3)
    )
    return [
        [usd_rotation[row][column] * scales[column] for column in range(3)] + [0.0]
        for row in range(3)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def test_scale_safe_rotation_extraction_preserves_yaw_and_unit_norm():
    yaw = -2.4494240864
    quaternion = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    matrix = _scaled_rotation_matrix(quaternion, (0.07, 0.07, 0.08))

    extracted = IsaacFixtureRuntime._rotation_quaternion_from_matrix(matrix)
    basis = IsaacFixtureRuntime._orthonormalized_rotation_basis(matrix)
    basis_quaternion = IsaacFixtureRuntime._quaternion_from_rotation_basis(basis)

    assert IsaacFixtureRuntime._quaternion_norm(extracted) == pytest.approx(1.0)
    assert _angular_distance(
        IsaacFixtureRuntime._yaw_from_components(basis_quaternion), -yaw
    ) == pytest.approx(0.0, abs=1e-12)
    assert _angular_distance(
        IsaacFixtureRuntime._yaw_from_components(extracted), yaw
    ) == pytest.approx(0.0, abs=1e-12)


def test_scale_safe_rotation_extraction_handles_parent_rotation_and_pi_wrap():
    local_yaw = math.pi - 0.02
    parent_yaw = 0.11
    yaw = local_yaw + parent_yaw
    quaternion = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    matrix = _scaled_rotation_matrix(quaternion, (0.07, 0.11, 0.08))

    extracted = IsaacFixtureRuntime._rotation_quaternion_from_matrix(matrix)

    assert _angular_distance(
        IsaacFixtureRuntime._yaw_from_components(extracted), yaw
    ) == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize(
    "yaw", (-2.4494240864, -math.pi / 2.0, math.pi / 2.0, math.pi - 1e-9)
)
def test_scale_safe_rotation_extraction_preserves_signed_yaw(yaw):
    quaternion = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    matrix = _scaled_rotation_matrix(quaternion, (0.07, 0.11, 0.08))

    extracted = IsaacFixtureRuntime._rotation_quaternion_from_matrix(matrix)

    assert IsaacFixtureRuntime._quaternion_norm(extracted) == pytest.approx(1.0)
    assert _angular_distance(
        IsaacFixtureRuntime._yaw_from_components(extracted), yaw
    ) == pytest.approx(0.0, abs=1e-9)


class _Prim:
    def __init__(self, path):
        self.path = path
        self.custom = {}
        self.attributes = {}

    def IsValid(self):
        return True

    def GetPath(self):
        return self.path

    def SetCustomDataByKey(self, key, value):
        self.custom[key] = value

    def GetCustomDataByKey(self, key):
        return self.custom.get(key)

    def GetAttribute(self, key):
        return self.attributes.get(key, SimpleNamespace(IsValid=lambda: False))

    def CreateAttribute(self, key, value_type):
        del value_type
        attribute = SimpleNamespace(value=None)
        attribute = SimpleNamespace(
            IsValid=lambda: True,
            Set=lambda value: (
                setattr(attribute, "value", value),
                self.attributes.__setitem__(key, attribute),
            ),
        )
        self.attributes[key] = attribute
        return attribute


class _Stage:
    def __init__(self):
        self.prims = {}

    def GetPrimAtPath(self, path):
        return self.prims.get(path, SimpleNamespace(IsValid=lambda: False))

    def Traverse(self):
        return list(self.prims.values())


class _CanonicalRigidBodyAPI:
    applied = []

    def __init__(self, prim):
        self.prim = prim

    @classmethod
    def Apply(cls, prim):
        cls.applied.append(prim.GetPath())
        return cls(prim)

    def CreateKinematicEnabledAttr(self):
        return self.prim.CreateAttribute("physics:kinematicEnabled", bool)


class _CanonicalPhysxRigidBodyAPI:
    @classmethod
    def Apply(cls, prim):
        return cls(prim)

    def __init__(self, prim):
        self.prim = prim

    def CreateDisableGravityAttr(self):
        return self.prim.CreateAttribute("physxRigidBody:disableGravity", bool)


def test_fixture_stability_uses_canonical_rigid_body_schema_for_kinematic_hold():
    stage = _Stage()
    stage.prims["/World/Validation/Cube"] = _Prim("/World/Validation/Cube")
    _CanonicalRigidBodyAPI.applied.clear()
    guard = IsaacFixtureRuntime(
        stage,
        rigid_body_api_factory=_CanonicalRigidBodyAPI.Apply,
    )
    guard._owned_path = "/World/Validation/Cube"
    guard.pre_capture_stability = guard._new_stability_guard("/World/Validation/Cube")

    assert guard.pre_capture_stability.hold_for_registration() is True
    assert _CanonicalRigidBodyAPI.applied == ["/World/Validation/Cube"]
    assert (
        stage.prims["/World/Validation/Cube"]
        .attributes["physics:kinematicEnabled"]
        .value
        is True
    )


def test_fixture_gravity_policy_uses_canonical_physx_schema():
    stage = _Stage()
    path = "/World/Validation/Cube"
    stage.prims[path] = _Prim(path)
    runtime = IsaacFixtureRuntime(
        stage,
        physx_rigid_body_api_factory=_CanonicalPhysxRigidBodyAPI.Apply,
    )
    runtime._owned_path = path
    runtime._set_owner(stage.prims[path])

    runtime.disable_gravity(path)

    assert stage.prims[path].attributes["physxRigidBody:disableGravity"].value is True


def test_fixture_gravity_enable_uses_canonical_physx_schema():
    stage = _Stage()
    path = "/World/Validation/Cube"
    stage.prims[path] = _Prim(path)
    runtime = IsaacFixtureRuntime(
        stage,
        physx_rigid_body_api_factory=_CanonicalPhysxRigidBodyAPI.Apply,
    )
    runtime._owned_path = path
    runtime._set_owner(stage.prims[path])

    runtime.enable_gravity(path)

    assert stage.prims[path].attributes["physxRigidBody:disableGravity"].value is False


def test_isaac_runtime_owns_one_fixture_and_registers_only_that_prim():
    stage = _Stage()
    registered = []

    def create_cube(path, pose, dimensions):
        del pose, dimensions
        stage.prims[path] = _Prim(path)

    def delete_prim(path):
        del stage.prims[path]

    runtime = IsaacFixtureRuntime(
        stage,
        rigid_body_api_factory=_CanonicalRigidBodyAPI.Apply,
        registry=SimpleNamespace(
            register=lambda path: registered.append(path),
            unregister=lambda path: None,
        ),
        cube_factory=create_cube,
        delete_prim=delete_prim,
    )

    runtime.spawn_cube(
        "/World/Validation/Cube", Pose(0.2, 0.0, 0.8), (0.14, 0.14, 0.08)
    )
    runtime.register_eligible("/World/Validation/Cube", "run-1")

    assert registered == ["/World/Validation/Cube"]
    with pytest.raises(RuntimeError, match="one fixture cube"):
        runtime.spawn_cube(
            "/World/Validation/Second", Pose(0.2, 0.0, 0.8), (0.14, 0.14, 0.08)
        )

    assert runtime.delete_prim("/World/Foreign/Cube") is False


def test_retired_fixture_stays_in_stage_gravity_disabled_and_not_capture_eligible():
    stage = _Stage()
    path = "/World/Validation/Cube"
    prim = _Prim(path)
    IsaacFixtureRuntime._set_owner(prim)
    stage.prims[path] = prim
    candidates = set()
    registry = SimpleNamespace(
        register=candidates.add,
        unregister=candidates.discard,
    )
    runtime = IsaacFixtureRuntime(
        stage,
        physx_rigid_body_api_factory=_CanonicalPhysxRigidBodyAPI.Apply,
        registry=registry,
    )
    runtime._owned_path = path

    assert runtime.retire_fixture_for_pending_delete(path)

    assert stage.GetPrimAtPath(path).IsValid()
    assert not candidates
    assert prim.attributes["physxRigidBody:disableGravity"].value is True


def test_isaac_runtime_replaces_stale_same_owner_fixture_before_spawn():
    stage = _Stage()
    path = "/World/Validation/Cube"
    stale = _Prim(path)
    IsaacFixtureRuntime._set_owner(stale)
    stage.prims[path] = stale
    deleted = []
    unregistered = []

    def create_cube(prim_path, pose, dimensions):
        del pose, dimensions
        stage.prims[prim_path] = _Prim(prim_path)

    def delete_prim(prim_path):
        deleted.append(prim_path)
        stage.prims.pop(prim_path, None)

    runtime = IsaacFixtureRuntime(
        stage,
        rigid_body_api_factory=_CanonicalRigidBodyAPI.Apply,
        registry=SimpleNamespace(
            register=lambda path: None,
            unregister=lambda path: unregistered.append(path),
        ),
        cube_factory=create_cube,
        delete_prim=delete_prim,
    )

    runtime.spawn_cube(path, Pose(0.2, 0.0, 0.8), (0.07, 0.07, 0.08))

    assert deleted == [path]
    assert unregistered == [path]
    assert stage.GetPrimAtPath(path).IsValid()
    assert IsaacFixtureRuntime._is_owned(stage.GetPrimAtPath(path))


def test_isaac_runtime_does_not_replace_foreign_fixture_at_canonical_path():
    stage = _Stage()
    path = "/World/Validation/Cube"
    stage.prims[path] = _Prim(path)
    deleted = []

    runtime = IsaacFixtureRuntime(
        stage,
        rigid_body_api_factory=_CanonicalRigidBodyAPI.Apply,
        cube_factory=lambda *args: None,
        delete_prim=lambda prim_path: deleted.append(prim_path),
    )

    with pytest.raises(RuntimeError, match="already occupied"):
        runtime.spawn_cube(path, Pose(0.2, 0.0, 0.8), (0.07, 0.07, 0.08))

    assert deleted == []


def test_isaac_runtime_does_not_replace_fixture_with_owned_grasp_joint():
    stage = _Stage()
    path = "/World/Validation/Cube"
    stale = _Prim(path)
    IsaacFixtureRuntime._set_owner(stale)
    stage.prims[path] = stale
    stage.prims[f"{path}/sf_scripted_grasp_joint"] = _Prim(
        f"{path}/sf_scripted_grasp_joint"
    )
    deleted = []

    runtime = IsaacFixtureRuntime(
        stage,
        rigid_body_api_factory=_CanonicalRigidBodyAPI.Apply,
        cube_factory=lambda *args: None,
        delete_prim=lambda prim_path: deleted.append(prim_path),
    )

    with pytest.raises(RuntimeError, match="grasp joint"):
        runtime.spawn_cube(path, Pose(0.2, 0.0, 0.8), (0.07, 0.07, 0.08))

    assert deleted == []


def test_isaac_runtime_treats_spawn_pose_as_world_coordinates():
    stage = _Stage()
    observed = []

    def create_cube(path, pose, dimensions):
        observed.append(pose)
        del dimensions
        stage.prims[path] = _Prim(path)

    runtime = IsaacFixtureRuntime(
        stage,
        world_to_base=(0.15, -0.15, 0.68, 0.0),
        rigid_body_api_factory=_CanonicalRigidBodyAPI.Apply,
        cube_factory=create_cube,
    )

    runtime.spawn_cube(
        "/World/Validation/Cube", Pose(0.44, 0.06, 0.86), (0.07, 0.07, 0.08)
    )

    assert observed[0] == Pose(0.44, 0.06, 0.86)


def test_fixture_stability_is_held_until_joint_transition_then_released():
    stage = _Stage()

    def create_cube(path, pose, dimensions):
        del pose, dimensions
        stage.prims[path] = _Prim(path)

    runtime = IsaacFixtureRuntime(
        stage,
        rigid_body_api_factory=_CanonicalRigidBodyAPI.Apply,
        registry=SimpleNamespace(
            register=lambda path: None, unregister=lambda path: None
        ),
        cube_factory=create_cube,
    )
    path = "/World/Validation/Cube"
    runtime.spawn_cube(path, Pose(0.2, 0.0, 0.8), (0.14, 0.14, 0.08))
    runtime.register_eligible(path, "run-1")

    assert runtime.pre_capture_stability.is_active is True
    assert stage.prims[path].attributes["physics:kinematicEnabled"].value is True

    assert runtime.pre_capture_stability.release_for_capture(path) is True
    assert runtime.pre_capture_stability.is_active is False
    assert stage.prims[path].attributes["physics:kinematicEnabled"].value is False


def test_fixture_stability_rejects_foreign_capture_transition():
    stage = _Stage()
    runtime = IsaacFixtureRuntime(
        stage,
        rigid_body_api_factory=_CanonicalRigidBodyAPI.Apply,
        registry=SimpleNamespace(
            register=lambda path: None, unregister=lambda path: None
        ),
        cube_factory=lambda path, pose, dimensions: stage.prims.__setitem__(
            path, _Prim(path)
        ),
    )
    runtime.spawn_cube(
        "/World/Validation/Cube", Pose(0.2, 0.0, 0.8), (0.14, 0.14, 0.08)
    )

    assert (
        runtime.pre_capture_stability.release_for_capture("/World/Foreign/Cube")
        is False
    )
    assert runtime.pre_capture_stability.is_active is True
