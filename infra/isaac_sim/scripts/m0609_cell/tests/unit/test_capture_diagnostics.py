import threading
import sys
import types
import json
from types import SimpleNamespace

import gripper_runtime.grasp_attachment as grasp_attachment
from gripper_runtime.capture_diagnostics import (
    BoundedCaptureDiagnosticWriter,
    CaptureDiagnosticRecord,
    IsaacGripperDiagnosticRecord,
    IsaacPhysXObserver,
)
from gripper_runtime.grasp_policy import CaptureCandidate, CaptureConfig
from gripper_runtime.session import GripperRuntimeSession


def test_release_record_keeps_command_local_initial_actuator_state():
    writer = BoundedCaptureDiagnosticWriter(
        "/unused", write_record=lambda _record: None
    )
    record = IsaacGripperDiagnosticRecord(
        writer, 7, 85.0, initial_actuator_state="opening"
    )

    assert record.data["events"][0]["kind"] == "actuator_state"
    assert record.data["events"][0]["payload"]["state"] == "opening"
    assert record.last_actuator_state == "opening"
    assert record.finalize("released") is True
    assert writer.close() is True


def test_release_record_writer_submit_failure_is_nonfatal():
    class _FailingWriter:
        def submit(self, _record):
            raise OSError("diagnostic storage unavailable")

    record = IsaacGripperDiagnosticRecord(_FailingWriter(), 8, 85.0)

    assert record.finalize("released") is False
    assert record.finalized is True


def test_simulation_stop_finalizes_active_release_and_resets_trace_state():
    events = []
    session = GripperRuntimeSession.__new__(GripperRuntimeSession)
    session.grasp_manager = SimpleNamespace(
        reset_for_simulation_stop=lambda: events.append("reset") or True,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="held"), sequence=12
        ),
        joint_owner=SimpleNamespace(has_owned_joint=lambda: True),
    )
    session._active_release_record = SimpleNamespace(
        finalize=lambda outcome, details: events.append((outcome, details))
    )
    session._last_release_joint_owned = True
    session._last_release_holding_state = "held"

    assert session.reset_for_simulation_stop() is True
    assert events[0][0] == "simulation_stop_before_release_confirmation"
    assert events[1] == "reset"
    assert session._active_release_record is None
    assert session._last_release_joint_owned is None
    assert session._last_release_holding_state is None


def test_writer_bounds_pending_records_and_preserves_submitted_snapshot():
    entered = threading.Event()
    release = threading.Event()
    written = []

    def write(record):
        if record["id"] == "first":
            entered.set()
            assert release.wait(2.0)
        written.append(record)

    writer = BoundedCaptureDiagnosticWriter(
        "/unused", max_pending=1, write_record=write
    )
    try:
        first = {"id": "first", "events": []}
        assert writer.submit(first)
        assert entered.wait(2.0)
        assert writer.submit({"id": "second"})
        assert not writer.submit({"id": "dropped"})
        assert writer.dropped_count == 1
    finally:
        release.set()
        writer.close()

    assert written == [{"id": "first", "events": []}, {"id": "second"}]
    assert not writer._thread.is_alive()
    assert writer.shutdown_status == "complete"
    assert writer.written_count == 2


def test_writer_reports_incomplete_shutdown_when_worker_exceeds_bound():
    entered = threading.Event()
    release = threading.Event()

    def write(_record):
        entered.set()
        release.wait(2.0)

    writer = BoundedCaptureDiagnosticWriter("/unused", write_record=write)
    assert writer.submit({"id": "slow"}) is True
    assert entered.wait(2.0)
    assert writer.close(timeout=0.01) is False
    assert writer.shutdown_status == "incomplete"
    release.set()
    writer._thread.join(2.0)
    assert not writer._thread.is_alive()


def test_writer_serializes_and_validates_only_on_worker_thread(monkeypatch):
    serialization_threads = []
    written = []
    original_dumps = json.dumps

    def track_dumps(*args, **kwargs):
        serialization_threads.append(threading.current_thread().name)
        return original_dumps(*args, **kwargs)

    monkeypatch.setattr(json, "dumps", track_dumps)
    writer = BoundedCaptureDiagnosticWriter(
        "/unused", write_record=lambda record: written.append(record)
    )
    record = CaptureDiagnosticRecord(writer, "goal-worker", "goal-worker")
    record.append("decision", {"answer": 42})
    assert record.finalize() is True
    assert serialization_threads == []
    assert writer.close() is True
    assert serialization_threads
    assert all(name == "capture-diagnostic-writer" for name in serialization_threads)
    assert written[0]["events"][0]["payload"]["answer"] == 42


def test_writer_json_serialization_failure_is_async_and_reported():
    writer = BoundedCaptureDiagnosticWriter("/unused")
    assert writer.submit({"bad": object()}) is True
    assert writer.close() is True
    assert writer.write_failure_count == 1
    assert writer.shutdown_status == "complete"


def test_writer_enforces_record_and_directory_byte_limits(tmp_path, monkeypatch):
    import gripper_runtime.capture_diagnostics as capture_diagnostics

    monkeypatch.setattr(capture_diagnostics, "MAX_CAPTURE_RECORD_BYTES", 1024)
    monkeypatch.setattr(capture_diagnostics, "MAX_CAPTURE_DIRECTORY_BYTES", 100)
    writer = BoundedCaptureDiagnosticWriter(tmp_path)
    assert writer.submit({"motion_goal_uuid": "goal-one", "payload": "x"})
    assert writer.submit({"motion_goal_uuid": "goal-two", "payload": "x"})
    assert writer.close()
    assert (tmp_path / "goal-one.json").is_file()
    assert not (tmp_path / "goal-two.json").exists()
    assert writer.write_failure_count == 1

    monkeypatch.setattr(capture_diagnostics, "MAX_CAPTURE_RECORD_BYTES", 16)
    oversized = BoundedCaptureDiagnosticWriter(tmp_path / "oversized")
    assert oversized.submit({"motion_goal_uuid": "goal-large", "payload": "x"})
    assert oversized.close()
    assert not (tmp_path / "oversized/goal-large.json").exists()
    assert oversized.write_failure_count == 1


def test_capture_record_caps_event_memory_and_keeps_final_outcome(tmp_path):
    written = []
    writer = BoundedCaptureDiagnosticWriter(
        tmp_path, write_record=lambda record: written.append(record)
    )
    record = CaptureDiagnosticRecord(writer, "goal-bound", "goal-bound", max_events=3)
    for value in range(20):
        record.append("observation", {"sample": value})
    record.set_outcome("unknown", "test", 4, 1.0)

    record.finalize()
    writer.close()

    assert record.data is None
    assert len(written[0]["events"]) == 3
    assert written[0]["events"][-1]["kind"] == "outcome"
    assert written[0]["coverage"]["events_dropped"] == 18


def test_physx_observer_reads_single_rigid_prim_dynamic_state(monkeypatch):
    rigid_api = object()
    stage_prim = types.SimpleNamespace(
        IsValid=lambda: True,
        HasAPI=lambda api: api is rigid_api,
    )
    stage = types.SimpleNamespace(GetPrimAtPath=lambda _path: stage_prim)
    state = types.SimpleNamespace(
        position=(1.0, 2.0, 3.0),
        orientation=(1.0, 0.0, 0.0, 0.0),
        linear_velocity=(0.1, 0.2, 0.3),
        angular_velocity=(0.4, 0.5, 0.6),
    )

    class _SingleRigidPrim:
        def __init__(self, prim_path, reset_xform_properties):
            assert prim_path in ("/World/TCP", "/World/RawPart")
            assert reset_xform_properties is False

        def initialize(self):
            pass

        def get_current_dynamic_state(self):
            return state

    modules = {
        "pxr": types.ModuleType("pxr"),
        "omni": types.ModuleType("omni"),
        "omni.usd": types.ModuleType("omni.usd"),
        "isaacsim": types.ModuleType("isaacsim"),
        "isaacsim.core": types.ModuleType("isaacsim.core"),
        "isaacsim.core.prims": types.ModuleType("isaacsim.core.prims"),
    }
    modules["pxr"].UsdPhysics = types.SimpleNamespace(RigidBodyAPI=rigid_api)
    modules["omni"].usd = modules["omni.usd"]
    modules["omni.usd"].get_context = lambda: types.SimpleNamespace(
        get_stage=lambda: stage
    )
    modules["isaacsim.core.prims"].SingleRigidPrim = _SingleRigidPrim
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    observer = IsaacPhysXObserver()
    observer.prepare(["/World/TCP", "/World/RawPart"])
    result = observer.observe_pair("/World/TCP", "/World/RawPart")

    assert result["source"] == "isaacsim.core.prims.SingleRigidPrim_physx"
    assert result["tcp"]["status"] == "observed", result
    assert result["tcp"]["pose_world"]["position_m"] == (1.0, 2.0, 3.0)
    assert result["tcp"]["linear_velocity_m_s"] == (0.1, 0.2, 0.3)
    assert result["rawpart"]["angular_velocity_rad_s"] == (0.4, 0.5, 0.6)


def test_successful_capture_writes_goal_correlated_observation_decision_outcome(
    tmp_path,
):
    class _Gripper:
        MODEL_ID = "robotiq_2f_85"
        state = types.SimpleNamespace(value="holding")

        def get_width(self):
            return 12.0

        def stop(self):
            self.state.value = "holding"

    class _CandidateSource:
        def query(self):
            return True, [
                CaptureCandidate(
                    "/World/RawPart",
                    True,
                    (0.0, 0.0, 0.0),
                    0.001,
                    0.5,
                    12.0,
                    shape="box",
                    tcp_stage_world_pose={
                        "status": "unavailable",
                        "reason": "fake source",
                    },
                    stage_world_pose={"status": "unavailable", "reason": "fake source"},
                )
            ]

    class _JointOwner:
        def __init__(self):
            self.created = False

        def has_owned_joint(self):
            return self.created

        def create(self, _path):
            self.created = True
            return True

        def owned_joint_valid(self):
            return self.created

        def set_ownership_validation(self, valid):
            self.validated = valid

    config = CaptureConfig((0.1, 0.1, 0.1))
    manager = grasp_attachment.GraspManager(
        gripper=_Gripper(),
        tcp_path="/World/TCP",
        config=config,
        candidate_source=_CandidateSource(),
        joint_owner=_JointOwner(),
    )
    manager.capture_reference_id = "motion-goal-42"
    writer = BoundedCaptureDiagnosticWriter(tmp_path)
    manager.set_diagnostic_observers(writer=writer)
    assert manager.arm_close_transaction("motion-goal-42")

    manager.update(1.0 / 60.0)
    manager.finalize_capture_diagnostic("four-step-window-unavailable-in-unit-test")
    writer.close()

    record = json.loads((tmp_path / "motion-goal-42.json").read_text())
    assert record["motion_goal_uuid"] == "motion-goal-42"
    assert record["capture_reference_id"] == "motion-goal-42"
    events = record["events"]
    assert any(
        event["payload"].get("phase") == "close_completion"
        and event["payload"]["close_completed"]
        for event in events
    )
    capture_decision = next(
        event["payload"]
        for event in events
        if event["kind"] == "decision"
        and event["payload"].get("policy") == "CapturePolicy.evaluate"
    )
    assert capture_decision["decision"] == "capture_eligible"
    assert capture_decision["selected_candidate"] == "/World/RawPart"
    assert any(
        event["payload"].get("phase") == "attach_immediately_before_joint_creation"
        and event["payload"]["pair"]["status"] == "unavailable"
        for event in events
    )
    assert record["events"][-1]["payload"]["holding_state"] == "held"


def test_session_records_four_distinct_physx_and_stage_step_observations():
    class _Record:
        def __init__(self):
            self.events = []
            self.data = {"coverage": {"physics_step_limit": 4}}

        def append(self, kind, payload, **metadata):
            self.events.append((kind, payload, metadata))

    class _CandidateSource:
        @staticmethod
        def _world_transform(path):
            return path

        @staticmethod
        def _pose_dict(path):
            return {"path": path, "source": "usd_stage_world"}

    class _Manager:
        tcp_path = "/World/TCP"
        candidate_source = _CandidateSource()
        capture_diagnostic_target_path = "/World/RawPart"

        def __init__(self):
            self.capture_diagnostic_record = _Record()
            self.finalized = False

        def holding_observation(self):
            return types.SimpleNamespace(state=types.SimpleNamespace(value="held"))

        def finalize_capture_diagnostic(self):
            self.finalized = True

    class _Observer:
        def observe_pair(self, _tcp, _rawpart):
            return {
                "source": "physx_rigid_body",
                "tcp": {
                    "status": "observed",
                    "pose_world": {
                        "position_m": (0, 0, 0),
                        "orientation_wxyz": (1, 0, 0, 0),
                    },
                    "linear_velocity_m_s": (0, 0, 0),
                    "angular_velocity_rad_s": (0, 0, 0),
                },
                "rawpart": {
                    "status": "observed",
                    "pose_world": {
                        "position_m": (1, 0, 0),
                        "orientation_wxyz": (1, 0, 0, 0),
                    },
                    "linear_velocity_m_s": (0, 0, 0),
                    "angular_velocity_rad_s": (0, 0, 0),
                },
            }

    session = GripperRuntimeSession.__new__(GripperRuntimeSession)
    session.grasp_manager = _Manager()
    record = session.grasp_manager.capture_diagnostic_record
    session.physx_observer = _Observer()
    session._physics_step_sequence = 10
    session._capture_physics_steps = 0
    session._previous_capture_relative = None

    for _ in range(4):
        session._physics_step_sequence += 1
        session._observe_capture_physics_step(1.0 / 60.0)

    events = record.events
    assert [event[2]["physics_step"] for event in events] == [11, 12, 13, 14]
    assert all(event[2]["dt_s"] == 1.0 / 60.0 for event in events)
    assert all(event[1]["pair"]["source"] == "physx_rigid_body" for event in events)
    assert all(
        event[1]["stage_world_pose"]["source"] == "usd_stage_world" for event in events
    )
    assert record.data["coverage"]["physics_steps_observed"] == 4
    assert session.grasp_manager.finalized is True


def test_failed_capture_record_retains_goal_uuid_and_unknown_reason(tmp_path):
    class _Gripper:
        state = types.SimpleNamespace(value="closing")

        def get_width(self):
            return 12.0

        def stop(self):
            self.state.value = "holding"

    class _CandidateSource:
        def query(self):
            return True, [
                CaptureCandidate(
                    "/World/RawPart", True, (0.06, 0.0, 0.0), 0.02, 20.0, 30.0
                )
            ]

    class _JointOwner:
        def has_owned_joint(self):
            return False

        def create(self, _path):
            raise AssertionError("outside candidate cannot attach")

    manager = grasp_attachment.GraspManager(
        gripper=_Gripper(),
        tcp_path="/World/TCP",
        config=CaptureConfig((0.1, 0.1, 0.1)),
        candidate_source=_CandidateSource(),
        joint_owner=_JointOwner(),
    )
    manager.capture_reference_id = "motion-goal-failed-7"
    writer = BoundedCaptureDiagnosticWriter(tmp_path)
    manager.set_diagnostic_observers(writer=writer)
    assert manager.arm_close_transaction("motion-goal-failed-7")

    manager.update(1.0 / 60.0)
    manager.gripper.state.value = "holding"
    manager.update(1.0 / 60.0)
    writer.close()

    record = json.loads((tmp_path / "motion-goal-failed-7.json").read_text())
    decisions = [
        event["payload"] for event in record["events"] if event["kind"] == "decision"
    ]
    assert record["motion_goal_uuid"] == "motion-goal-failed-7"
    assert decisions[-1]["decision"] == "no_capture"
    assert (
        decisions[-1]["assessments"][0]["rejection_reason"] == "outside_capture_volume"
    )
    assert record["events"][-1]["payload"]["holding_state"] == "unknown"
    assert record["events"][-1]["payload"]["reason"] == "outside_capture_volume"
    assert record["events"][-1]["payload"]["capture_decision"] == "no_capture"
    assert record["events"][-1]["payload"]["capture_result"] == "rejected"


def test_capture_rejection_warns_once_after_close_finishes_and_matches_policy_reason(
    tmp_path, caplog, monkeypatch
):
    class _Gripper:
        state = types.SimpleNamespace(value="closing")

        def get_width(self):
            return 12.0

        def stop(self):
            self.state.value = "holding"

    class _CandidateSource:
        def query(self):
            return True, [
                CaptureCandidate(
                    "/World/RawPart", True, (0.06, 0.0, 0.0), 0.02, 20.0, 12.0
                )
            ]

    class _JointOwner:
        def has_owned_joint(self):
            return False

    manager = grasp_attachment.GraspManager(
        gripper=_Gripper(),
        tcp_path="/World/TCP",
        config=CaptureConfig((0.1, 0.1, 0.1)),
        candidate_source=_CandidateSource(),
        joint_owner=_JointOwner(),
    )
    manager.capture_reference_id = "motion-goal-closing"
    writer = BoundedCaptureDiagnosticWriter(tmp_path)
    manager.set_diagnostic_observers(writer=writer)
    assert manager.arm_close_transaction("motion-goal-closing")

    monkeypatch.setattr(grasp_attachment.LOGGER, "propagate", True)
    caplog.set_level("WARNING", logger="gripper_runtime.grasp_attachment")
    manager.update(1.0 / 60.0)
    manager.update(1.0 / 60.0)
    assert not [r for r in caplog.records if "Gripper could not capture" in r.message]

    manager.gripper.state.value = "holding"
    manager.update(1.0 / 60.0)
    writer.close()

    warnings = [r for r in caplog.records if "Gripper could not capture" in r.message]
    assert len(warnings) == 1
    assert "outside_capture_volume" in warnings[0].message
    record = json.loads((tmp_path / "motion-goal-closing.json").read_text())
    decision = next(
        event["payload"]
        for event in reversed(record["events"])
        if event["kind"] == "decision"
    )
    outcome = record["events"][-1]["payload"]
    assert decision["failure_reason"] == "outside_capture_volume"
    assert outcome["reason"] == "outside_capture_volume"
    assert outcome["capture_decision"] == "no_capture"
    assert outcome["capture_result"] == "rejected"
