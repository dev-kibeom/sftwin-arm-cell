"""Unit tests for the ROS Humble side of the Isaac registration handoff."""

import importlib.util
import json
import stat
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts/register_pnp_fixture.py"
PREFLIGHT = Path(__file__).parents[1] / "scripts/pnp_validation_preflight.py"


def test_registration_helper_is_packaged_as_executable():
    assert stat.S_IMODE(SCRIPT.stat().st_mode) & stat.S_IXUSR
    assert stat.S_IMODE(PREFLIGHT.stat().st_mode) & stat.S_IXUSR


def _module():
    spec = importlib.util.spec_from_file_location("register_pnp_fixture", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _handoff():
    return {
        "schema_version": "wu14-pnp-registration/v1",
        "status": "FIXTURE_READY_FOR_REGISTRATION",
        "run_id": "run-1",
        "target_id": "cube_profile",
        "request": {
            "run_id": "run-1",
            "target_id": "cube_profile",
            "pose": {
                "frame_id": "base_link",
                "x": 0.2,
                "y": -0.1,
                "z": 0.84,
                "yaw": 0.25,
            },
            "has_target_yaw": True,
            "ttl_s": 30.0,
        },
        "fixture": {
            "prim_identity": "/World/SFTwin/ValidationFixture/Cube",
            "seed": 17,
            "dimensions_m": [0.07, 0.07, 0.08],
        },
        "profile_reference": "profile.yaml",
    }


def test_request_mapping_preserves_context_pose_frame_and_ttl(monkeypatch):
    module = _module()

    class Message:
        def __init__(self):
            self.header = SimpleNamespace(frame_id="", stamp=None)
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(z=0.0, w=0.0),
            )

    class Duration:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    monkeypatch.setitem(
        sys.modules, "geometry_msgs.msg", SimpleNamespace(PoseStamped=Message)
    )
    monkeypatch.setitem(
        sys.modules, "builtin_interfaces.msg", SimpleNamespace(Duration=Duration)
    )

    class Request:
        pass

    request = module.request_from_handoff(_handoff(), Request)

    assert request.target_id == "cube_profile"
    assert request.run_id == "run-1"
    assert request.pose.header.frame_id == "base_link"
    assert request.pose.pose.position.z == pytest.approx(0.84)
    assert request.has_target_yaw is True
    assert request.ttl.sec == 30
    assert request.ttl.nanosec == 0


def test_receipt_requires_acceptance_and_positive_receipt_fields():
    module = _module()

    assert (
        module.validate_receipt(
            SimpleNamespace(
                accepted=True, receipt_sequence=1, receipt_time_ns=123, diagnostic=""
            )
        ).receipt_sequence
        == 1
    )
    with pytest.raises(RuntimeError, match="receipt rejected"):
        module.validate_receipt(
            SimpleNamespace(
                accepted=False,
                receipt_sequence=0,
                receipt_time_ns=123,
                diagnostic="bad",
            )
        )


def test_handoff_loader_rejects_non_ready_or_wrong_frame(tmp_path):
    module = _module()
    path = tmp_path / "handoff.json"
    data = _handoff()
    data["request"]["pose"]["frame_id"] = "world"
    path.write_text(json.dumps(data))

    with pytest.raises(ValueError, match="incomplete"):
        module.load_handoff(path)


def test_accepted_registration_state_is_written_atomically_for_runtime_bridge(tmp_path):
    module = _module()
    destination = tmp_path / "planning_scene_state.json"

    written = module.write_planning_scene_state(_handoff(), destination)

    assert written == destination
    assert json.loads(destination.read_text())["target_id"] == "cube_profile"
    assert not list(tmp_path.glob("*.tmp"))
