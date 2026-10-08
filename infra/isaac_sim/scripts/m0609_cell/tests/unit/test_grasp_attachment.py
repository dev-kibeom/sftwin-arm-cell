from dataclasses import replace
from types import SimpleNamespace

import pytest

from gripper_runtime.grasp_attachment import (
    CaptureReferenceConfig,
    GraspManager,
    HoldingState,
    StageCaptureQuery,
    UsdFixedJointOwner,
)
from gripper_runtime.grasp_policy import (
    CaptureCandidate,
    CaptureConfig,
)


def capture_config():
    return CaptureConfig(
        capture_volume_dimensions_m=(0.10, 0.10, 0.10),
        position_tolerance_m=0.01,
        orientation_tolerance_deg=5.0,
        expected_contact_width_mm=12.0,
        contact_width_tolerance_mm=1.0,
    )


def valid_candidate(path="/World/Cube"):
    return CaptureCandidate(
        prim_path=path,
        eligible=True,
        tcp_local_position_m=(0.0, 0.0, 0.0),
        position_error_m=0.005,
        orientation_error_deg=2.0,
        measured_opening_mm=12.0,
    )


class _BasisTransform:
    def __init__(
        self,
        x_axis=(1.0, 0.0, 0.0),
        y_axis=(0.0, 1.0, 0.0),
        z_axis=(0.0, 0.0, 1.0),
    ):
        self.axes = (x_axis, y_axis, z_axis)

    def TransformDir(self, axis):
        return tuple(
            sum(self.axes[index][component] * axis[index] for index in range(3))
            for component in range(3)
        )


def test_orientation_ignores_raw_frame_rotation_when_grasp_axes_align():
    tcp = _BasisTransform()
    cube = _BasisTransform(
        x_axis=(0.0, 1.0, 0.0),
        y_axis=(-1.0, 0.0, 0.0),
    )

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) > 5.0


def test_orientation_accepts_ninety_degree_cube_top_face_symmetry():
    tcp = _BasisTransform(z_axis=(0.0, 0.0, -1.0))
    cube = _BasisTransform(
        x_axis=(0.0, 1.0, 0.0),
        y_axis=(-1.0, 0.0, 0.0),
    )

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) <= 5.0


def test_orientation_rejects_approach_axis_outside_tolerance():
    tcp = _BasisTransform(z_axis=(0.0, 0.70710678, 0.70710678))
    cube = _BasisTransform()

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) > 5.0


def test_orientation_rejects_reversed_top_approach():
    tcp = _BasisTransform()
    cube = _BasisTransform()

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) > 5.0


class _PoseTransform(_BasisTransform):
    def __init__(self, position=(0.0, 0.0, 0.0), **kwargs):
        super().__init__(**kwargs)
        self.position = position

    def ExtractTranslation(self):
        return self.position


def test_canonical_capture_reference_uses_top_surface_and_grasp_offset():
    cube = _PoseTransform(position=(1.0, 2.0, 3.0))
    config = CaptureReferenceConfig(
        fixture_dimensions_m=(0.07, 0.07, 0.08),
        observation_to_grasp_tcp_m=-0.0025,
        insertion_axis_tcp=(0.0, 0.0, 1.0),
    )

    reference = StageCaptureQuery._canonical_capture_reference(cube, config)

    assert reference.position == (1.0, 2.0, 3.0375)
    assert reference.top_normal == (0.0, 0.0, 1.0)


def test_capture_reference_config_rejects_invalid_geometry():
    with pytest.raises(ValueError):
        CaptureReferenceConfig(
            fixture_dimensions_m=(0.07, 0.07, 0.0),
            observation_to_grasp_tcp_m=-0.0025,
            insertion_axis_tcp=(0.0, 0.0, 1.0),
        ).validate()


def test_orientation_rejects_closing_axis_outside_tolerance():
    tcp = _BasisTransform(x_axis=(0.0, 0.70710678, 0.70710678))
    cube = _BasisTransform()

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) > 5.0


class FakeGripper:
    def __init__(self):
        self.state = SimpleNamespace(value="closing")
        self.width_mm = 12.0
        self.stop_calls = 0

    def get_width(self):
        return self.width_mm

    def stop(self):
        self.stop_calls += 1
        self.state.value = "holding"

    def open(self, duration=None):
        del duration
        self.state.value = "opening"


class FakeCandidateSource:
    def __init__(self, candidates):
        self.candidates = candidates
        self.runtime_available = True

    def query(self):
        return self.runtime_available, list(self.candidates)


class FakeJointOwner:
    def __init__(self):
        self.owned_path = None
        self.created = []
        self.removed = []
        self.foreign_paths = {"/World/ForeignJoint"}
        self.valid = True
        self.target_exists = True
        self.remove_result = True

    def create(self, target_path):
        self.owned_path = f"{target_path}/sf_scripted_grasp_joint"
        self.created.append((self.owned_path, target_path))
        return self.owned_path

    def remove_owned(self):
        if self.owned_path is not None:
            self.removed.append(self.owned_path)
            if self.remove_result:
                self.owned_path = None
            return self.remove_result
        return False

    def owned_joint_valid(self):
        return self.valid and self.owned_path is not None and self.target_exists

    def has_owned_joint(self):
        return self.owned_path is not None


class FakePreCaptureStability:
    def __init__(self, joint_owner, result=True):
        self.joint_owner = joint_owner
        self.result = result
        self.calls = []

    def release_for_capture(self, target_path):
        self.calls.append((target_path, self.joint_owner.has_owned_joint()))
        return self.result


class ExistingUnownedJointOwner(FakeJointOwner):
    def __init__(self):
        super().__init__()
        self.existing_joint = True

    def create(self, target_path):
        del target_path
        return False


def test_existing_unowned_joint_does_not_silently_report_held():
    source = FakeCandidateSource([valid_candidate()])
    joints = ExistingUnownedJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_shutdown_cleanup_failure_invalidates_runtime_and_preserves_joint():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    joints.remove_result = False

    assert instance.shutdown() is False
    assert joints.has_owned_joint() is True
    assert instance.ready is False
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_repeated_close_open_cycle_reaches_held_and_released_deterministically():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    unknown = instance.holding_observation()
    assert unknown.sequence > 0
    instance.update(0.01)
    held = instance.holding_observation()
    assert held.state is HoldingState.HELD
    assert held.sequence > unknown.sequence

    instance.release(open_gripper=False)
    released = instance.holding_observation()
    assert released.state is HoldingState.RELEASED
    assert released.sequence > held.sequence

    instance.gripper.state.value = "closing"
    instance.arm_close_transaction()
    instance.update(0.01)
    held_again = instance.holding_observation()
    assert held_again.state is HoldingState.HELD
    assert held_again.sequence > released.sequence


def test_holding_sequence_changes_only_when_semantic_state_changes():
    instance = manager(FakeCandidateSource([valid_candidate()]), FakeJointOwner())

    instance.update(0.01)
    held = instance.holding_observation()
    assert held.state is HoldingState.HELD

    instance.update(0.01)
    repeated_held = instance.holding_observation()
    assert repeated_held.state is HoldingState.HELD
    assert repeated_held.sequence == held.sequence

    instance.release(open_gripper=False)
    released = instance.holding_observation()
    assert released.state is HoldingState.RELEASED
    assert released.sequence > repeated_held.sequence

    instance.ready = False
    instance.update(0.01)
    unknown = instance.holding_observation()
    assert unknown.state is HoldingState.UNKNOWN
    assert unknown.sequence > released.sequence


def test_runtime_readiness_change_advances_holding_sequence():
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    ready_unknown = instance.holding_observation()

    instance.ready = False
    instance.update(0.01)
    unavailable_unknown = instance.holding_observation()

    assert unavailable_unknown.state is HoldingState.UNKNOWN
    assert unavailable_unknown.sequence > ready_unknown.sequence


def test_applied_gripper_command_starts_new_observation_even_when_state_is_same():
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    instance.gripper.state.value = "holding"
    instance.update(0.01)
    released_before_command = instance.holding_observation()
    assert released_before_command.state is HoldingState.RELEASED

    instance.note_gripper_command_applied()
    released_after_command = instance.holding_observation()

    assert released_after_command.state is HoldingState.RELEASED
    assert released_after_command.sequence > released_before_command.sequence


def test_restored_holding_sequence_advances_past_previous_runtime_value():
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)

    instance.restore_holding_sequence(24614)
    instance.gripper.state.value = "holding"
    instance.update(0.01)

    observation = instance.holding_observation()
    assert observation.state is HoldingState.RELEASED
    assert observation.sequence == 24615


_DEFAULT_CONFIG = object()


def manager(source, joints, config=_DEFAULT_CONFIG, stability=None, arm=True):
    instance = GraspManager(
        gripper=FakeGripper(),
        tcp_path="/World/TCP",
        config=capture_config() if config is _DEFAULT_CONFIG else config,
        candidate_source=source,
        joint_owner=joints,
        pre_capture_stability=stability,
    )
    if arm:
        instance.arm_close_transaction()
    return instance


class _Prim:
    def __init__(
        self,
        valid=True,
        type_name="",
        path=None,
        body0=None,
        body1=None,
    ):
        self.valid = valid
        self.type_name = type_name
        self.path = path
        self.body0 = body0
        self.body1 = body1

    def IsValid(self):
        return self.valid

    def GetTypeName(self):
        return self.type_name

    def GetName(self):
        return str(self.path).rsplit("/", 1)[-1]

    def GetPath(self):
        return self.path

    def GetRelationship(self, name):
        return SimpleNamespace(
            GetTargets=lambda: [self.body0 if name == "physics:body0" else self.body1]
        )


class _Stage:
    def __init__(self, keep_removed_joint=False):
        self.prims = {
            "/World/TCP": _Prim(),
            "/World/Cube": _Prim(),
            "/World/Cube/sf_scripted_grasp_joint": _Prim(),
        }
        self.keep_removed_joint = keep_removed_joint
        self.removed_paths = []

    def GetPrimAtPath(self, path):
        return self.prims.get(str(path), _Prim(False))

    def Traverse(self):
        return tuple(self.prims.values())

    def RemovePrim(self, path):
        path = str(path)
        self.removed_paths.append(path)
        if not self.keep_removed_joint:
            self.prims.pop(path, None)


def _owned_stage_owner(keep_removed_joint=False):
    stage = _canonical_joint_stage(keep_removed_joint=keep_removed_joint)
    owner = UsdFixedJointOwner(stage, "/World/TCP")
    owner._owned_joint_path = "/World/Cube/sf_scripted_grasp_joint"
    owner._target_path = "/World/Cube"
    return owner, stage


def _canonical_joint_stage(keep_removed_joint=False, body0="/World/TCP"):
    stage = _Stage(keep_removed_joint=keep_removed_joint)
    stage.prims["/World/Cube/sf_scripted_grasp_joint"] = _Prim(
        type_name="PhysicsFixedJoint",
        path="/World/Cube/sf_scripted_grasp_joint",
        body0=body0,
        body1="/World/Cube",
    )
    return stage


def test_unowned_canonical_joint_is_reconciled_from_stage_signature():
    stage = _canonical_joint_stage()
    owner = UsdFixedJointOwner(stage, "/World/TCP")

    assert owner.reconcile_stale() is True
    assert stage.removed_paths == ["/World/Cube/sf_scripted_grasp_joint"]


def test_foreign_fixed_joint_is_preserved_during_reconciliation():
    stage = _canonical_joint_stage(body0="/World/ForeignTcp")
    owner = UsdFixedJointOwner(stage, "/World/TCP")

    assert owner.reconcile_stale() is False
    assert stage.removed_paths == []
    assert stage.GetPrimAtPath("/World/Cube/sf_scripted_grasp_joint").IsValid()


def test_reconciliation_failure_is_reported_without_clearing_state():
    stage = _canonical_joint_stage(keep_removed_joint=True)
    owner = UsdFixedJointOwner(stage, "/World/TCP")

    assert owner.reconcile_stale() is False
    assert stage.removed_paths == ["/World/Cube/sf_scripted_grasp_joint"]
    assert stage.GetPrimAtPath("/World/Cube/sf_scripted_grasp_joint").IsValid()


def test_usd_owner_clears_metadata_only_after_stage_confirms_joint_absence():
    owner, stage = _owned_stage_owner()

    assert owner.remove_owned() is True
    assert stage.removed_paths == ["/World/Cube/sf_scripted_grasp_joint"]
    assert owner.has_owned_joint() is False


def test_usd_owner_keeps_ownership_when_joint_remains_after_remove_request():
    owner, stage = _owned_stage_owner(keep_removed_joint=True)

    assert owner.remove_owned() is False
    assert stage.removed_paths == ["/World/Cube/sf_scripted_grasp_joint"]
    assert owner.has_owned_joint() is True
    assert owner.owned_joint_path == "/World/Cube/sf_scripted_grasp_joint"


def test_valid_candidate_creates_one_owned_joint_and_reports_held():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    assert len(joints.created) == 1
    assert instance.holding_observation().state is HoldingState.HELD
    assert instance.gripper.stop_calls == 1


def test_close_completion_holding_tick_still_evaluates_capture():
    source = FakeCandidateSource([replace(valid_candidate(), measured_opening_mm=20.0)])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)
    assert joints.created == []

    source.candidates = [valid_candidate()]
    instance.gripper.state.value = "holding"
    instance.update(0.01)

    assert len(joints.created) == 1
    assert instance.holding_observation().state is HoldingState.HELD


def test_closing_no_capture_is_transient_without_new_holding_observation():
    source = FakeCandidateSource([replace(valid_candidate(), measured_opening_mm=20.0)])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    baseline = instance.holding_observation().sequence
    instance.update(0.01)
    instance.update(0.01)

    assert instance.holding_observation().sequence == baseline
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert joints.created == []


def test_pending_closing_then_eligible_capture_publishes_one_fresh_held():
    source = FakeCandidateSource([replace(valid_candidate(), measured_opening_mm=20.0)])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    baseline = instance.holding_observation().sequence
    instance.update(0.01)
    source.candidates = [valid_candidate()]
    instance.update(0.01)

    observation = instance.holding_observation()
    assert observation.sequence == baseline + 1
    assert observation.state is HoldingState.HELD


def test_terminal_closing_capture_error_publishes_fresh_unknown():
    source = FakeCandidateSource(
        [valid_candidate("/World/CubeA"), valid_candidate("/World/CubeB")]
    )
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    observation = instance.holding_observation()
    assert observation.sequence == 1
    assert observation.state is HoldingState.UNKNOWN
    assert joints.created == []


def test_unknown_runtime_during_closing_publishes_fresh_unknown():
    source = FakeCandidateSource([valid_candidate()])
    source.runtime_available = False
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    observation = instance.holding_observation()
    assert observation.sequence == 1
    assert observation.state is HoldingState.UNKNOWN
    assert joints.created == []


def test_closing_without_explicit_transaction_arm_does_not_capture():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)

    instance.update(0.01)

    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_terminal_close_without_capture_disarms_and_does_not_retry():
    source = FakeCandidateSource([replace(valid_candidate(), measured_opening_mm=20.0)])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    baseline = instance.holding_observation().sequence
    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert instance.holding_observation().sequence == baseline

    source.candidates = [valid_candidate()]
    instance.update(0.01)

    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_new_close_transaction_can_supersede_expired_close_failure():
    source = FakeCandidateSource([replace(valid_candidate(), measured_opening_mm=20.0)])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.arm_close_transaction()
    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)

    source.candidates = [valid_candidate()]
    instance.gripper.state.value = "closing"
    instance.arm_close_transaction()
    instance.update(0.01)

    assert len(joints.created) == 1
    assert instance.holding_observation().state is HoldingState.HELD


def test_terminal_failure_does_not_retry_while_closing_until_explicit_rearm():
    source = FakeCandidateSource([replace(valid_candidate(), measured_opening_mm=20.0)])
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)
    instance.arm_close_transaction()

    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN

    instance.gripper.state.value = "closing"
    source.candidates = [valid_candidate()]
    instance.update(0.01)
    instance.update(0.01)
    assert joints.created == []

    instance.arm_close_transaction()
    instance.update(0.01)

    assert len(joints.created) == 1
    assert instance.holding_observation().state is HoldingState.HELD


def test_capture_releases_stability_only_after_adapter_joint_creation():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    stability = FakePreCaptureStability(joints)
    instance = manager(source, joints, stability=stability)

    instance.update(0.01)

    assert stability.calls == [("/World/Cube", True)]
    assert instance.holding_observation().state is HoldingState.HELD


def test_stability_release_failure_removes_joint_and_never_reports_held():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    stability = FakePreCaptureStability(joints, result=False)
    instance = manager(source, joints, stability=stability)

    instance.update(0.01)

    assert joints.removed == ["/World/Cube/sf_scripted_grasp_joint"]
    assert not joints.has_owned_joint()
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert not instance.is_grasped
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_stability_release_cleanup_failure_fails_closed_without_held():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    joints.remove_result = False
    stability = FakePreCaptureStability(joints, result=False)
    instance = manager(source, joints, stability=stability)

    instance.update(0.01)

    assert joints.has_owned_joint()
    assert instance.ready is False
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert not instance.is_grasped


def test_multiple_candidates_report_unknown_without_joint_creation():
    source = FakeCandidateSource(
        [valid_candidate("/World/CubeA"), valid_candidate("/World/CubeB")]
    )
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_open_removes_only_owned_joint_and_reports_released():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)

    instance.release(open_gripper=False)

    assert len(joints.removed) == 1
    assert joints.foreign_paths == {"/World/ForeignJoint"}
    assert instance.holding_observation().state is HoldingState.RELEASED


def test_joint_remaining_after_remove_request_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    joints.remove_result = False

    instance.gripper.open()
    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert joints.has_owned_joint() is True


def test_missing_owned_joint_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    joints.valid = False

    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_unexpected_owned_joint_relation_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    joints.valid = False

    instance.gripper.open()
    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert joints.removed == []


def test_unavailable_stage_query_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    source.runtime_available = False
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_opening_transition_removes_owned_joint_and_reports_released():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)

    instance.gripper.open()
    instance.update(0.01)

    assert len(joints.removed) == 1
    assert joints.foreign_paths == {"/World/ForeignJoint"}
    assert instance.holding_observation().state is HoldingState.RELEASED


def test_target_disappearance_during_open_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)

    joints.target_exists = False
    instance.gripper.open()
    instance.update(0.01)

    assert joints.removed == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_runtime_unavailable_during_open_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)

    source.runtime_available = False
    instance.gripper.open()
    instance.update(0.01)

    assert joints.removed == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_missing_capture_configuration_leaves_adapter_unready():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints, config=None)

    assert instance.ready is False
    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_invalid_capture_configuration_leaves_adapter_unready():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    invalid = capture_config()
    invalid = CaptureConfig(
        capture_volume_dimensions_m=(0.0, 0.10, 0.10),
        position_tolerance_m=invalid.position_tolerance_m,
        orientation_tolerance_deg=invalid.orientation_tolerance_deg,
        expected_contact_width_mm=invalid.expected_contact_width_mm,
        contact_width_tolerance_mm=invalid.contact_width_tolerance_mm,
    )
    instance = manager(source, joints, config=invalid)

    assert instance.ready is False
    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_valid_explicit_capture_configuration_marks_adapter_ready():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints, config=capture_config())

    assert instance.ready is True
