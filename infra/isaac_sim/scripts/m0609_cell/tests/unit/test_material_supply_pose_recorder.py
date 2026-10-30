import json
from pathlib import Path
from types import SimpleNamespace

from scene_builder.material_supply_adapter import _world_pose_to_local_transform
from scene_builder.material_supply_pose_recorder import (
    RawPartPoseRecorder,
)


class _Attribute:
    def __init__(self, value):
        self.value = value

    def Get(self):
        return self.value


class _Prim:
    def __init__(
        self,
        path,
        delivery="delivery-a",
        sequence=0,
        dimensions=(0.05, 0.04, 0.03),
        mass=0.12,
    ):
        self.path = path
        self.attributes = {
            "sf_twin:delivery_id": _Attribute(delivery),
            "sf_twin:delivery_sequence": _Attribute(sequence),
            "sf_twin:dimensions_m": _Attribute(dimensions),
            "physics:mass": _Attribute(mass),
        }

    def IsValid(self):
        return True

    def GetName(self):
        return self.path.rsplit("/", 1)[-1]

    def GetPath(self):
        return self.path

    def GetAttribute(self, name):
        return self.attributes.get(name)


class _Stage:
    def __init__(self):
        self.prims = []

    def Traverse(self):
        return iter(self.prims)


class _PhysX:
    def __init__(self):
        self.states = {}

    def get_rigidbody_transformation(self, path):
        return self.states.get(path, {"ret_val": False})


def _pose(x=0.0, quaternion=(0.0, 0.0, 0.0, 1.0)):
    return {"ret_val": True, "position": (x, 0.0, 0.0), "rotation": quaternion}


def _events(path: Path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_recorder_writes_initial_pose_and_suppresses_still_pose(tmp_path):
    stage = _Stage()
    prim = _Prim(
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_a/RawPart_0000"
    )
    stage.prims = [prim]
    physx = _PhysX()
    physx.states[prim.path] = _pose()
    recorder = RawPartPoseRecorder(stage, tmp_path, "live-session-a")

    recorder.sample(physx, 2.0, now=0.0)
    recorder.sample(physx, 2.2, now=0.2)

    events = _events(next(tmp_path.glob("*.jsonl")))
    assert [event["event"] for event in events] == ["session_started", "created"]
    assert events[1]["simulation_time_ns"] == 2_000_000_000
    assert events[1]["position_xyz"] == [0.0, 0.0, 0.0]
    assert events[1]["orientation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert events[1]["dimensions_m"] == [0.05, 0.04, 0.03]
    assert events[1]["mass_kg"] == 0.12
    assert events[0]["live_test_session_id"] == "live-session-a"
    assert events[1]["live_test_session_id"] == "live-session-a"
    assert events[1]["frame_id"] == "world"
    assert events[1]["observation_point"] == "physx_rigidbody_pose"
    assert events[1]["simulation_epoch"] == 0


def test_recorder_uses_one_mm_and_one_degree_thresholds(tmp_path):
    stage = _Stage()
    prim = _Prim(
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_a/RawPart_0000"
    )
    stage.prims = [prim]
    physx = _PhysX()
    physx.states[prim.path] = _pose()
    recorder = RawPartPoseRecorder(stage, tmp_path)
    recorder.sample(physx, 0.0, now=0.0)

    physx.states[prim.path] = _pose(x=0.0008)
    recorder.sample(physx, 0.2, now=0.2)
    physx.states[prim.path] = _pose(x=0.0011)
    recorder.sample(physx, 0.4, now=0.4)
    physx.states[prim.path] = _pose(
        x=0.0011,
        quaternion=(0.0, 0.0, 0.009599721, 0.999953923),
    )
    recorder.sample(physx, 0.6, now=0.6)
    physx.states[prim.path] = _pose(
        x=0.0011,
        quaternion=(0.0, 0.0, 0.017452406, 0.999847695),
    )
    recorder.sample(physx, 0.8, now=0.8)

    events = _events(next(tmp_path.glob("*.jsonl")))
    assert [event["event"] for event in events] == [
        "session_started",
        "created",
        "pose_changed",
        "pose_changed",
    ]
    assert events[2]["position_xyz"][0] == 0.0011
    assert events[3]["orientation_xyzw"][2] > 0.017


def test_recorder_tracks_create_remove_and_reconstructs_multi_part_layout(tmp_path):
    stage = _Stage()
    first = _Prim(
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_a/RawPart_0000"
    )
    stage.prims = [first]
    physx = _PhysX()
    physx.states[first.path] = _pose(0.25)
    recorder = RawPartPoseRecorder(stage, tmp_path)
    recorder.sample(physx, 1.0, now=0.0)

    second = _Prim(
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_b/RawPart_0000",
        "delivery-b",
    )
    stage.prims.append(second)
    physx.states[second.path] = _pose(0.75, (0.0, 0.0, 0.70710678, 0.70710678))
    recorder.sample(physx, 1.2, now=0.2)

    stage.prims.remove(first)
    recorder.sample(physx, 1.4, now=0.4)

    path = next(tmp_path.glob("*.jsonl"))
    assert [event["event"] for event in _events(path)] == [
        "session_started",
        "created",
        "created",
        "removed",
    ]
    created = [event for event in _events(path) if event["event"] == "created"]
    assert len(created) == 2
    assert created[1]["position_xyz"] == [0.75, 0.0, 0.0]
    assert created[1]["orientation_xyzw"] == [
        0.0,
        0.0,
        0.70710678,
        0.70710678,
    ]
    assert _events(path)[-1]["event"] == "removed"


def test_incomplete_physx_sample_does_not_emit_false_removal(tmp_path):
    stage = _Stage()
    first = _Prim(
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_a/RawPart_0000"
    )
    second = _Prim(
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_b/RawPart_0000",
        "delivery-b",
    )
    stage.prims = [first]
    physx = _PhysX()
    physx.states[first.path] = _pose()
    recorder = RawPartPoseRecorder(stage, tmp_path)
    recorder.sample(physx, 0.0, now=0.0)

    stage.prims.append(second)
    recorder.sample(physx, 0.2, now=0.2)
    events = _events(next(tmp_path.glob("*.jsonl")))
    assert [event["event"] for event in events] == ["session_started", "created"]


def test_simulation_time_rollback_starts_a_distinct_session(tmp_path):
    stage = _Stage()
    prim = _Prim(
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/Delivery_a/RawPart_0000"
    )
    stage.prims = [prim]
    physx = _PhysX()
    physx.states[prim.path] = _pose(0.1)
    recorder = RawPartPoseRecorder(stage, tmp_path)
    recorder.sample(physx, 4.0, now=0.0)
    physx.states[prim.path] = _pose(0.8)
    recorder.sample(physx, 0.5, now=0.2)

    sessions = list(tmp_path.glob("*.jsonl"))
    assert len(sessions) == 2
    assert all(
        [event["event"] for event in _events(path)] == ["session_started", "created"]
        for path in sessions
    )
    epochs = sorted(
        event["simulation_epoch"]
        for path in sessions
        for event in _events(path)
        if event["event"] == "created"
    )
    assert epochs == [0, 1]


class _Matrix:
    def __init__(self, position=(0.0, 0.0, 0.0), orientation=None):
        self.position = position
        self.orientation = orientation

    def SetRotate(self, orientation):
        self.orientation = orientation

    def SetTranslateOnly(self, position):
        self.position = tuple(position)

    def __mul__(self, other):
        return _Matrix(
            tuple(a + b for a, b in zip(self.position, other.position)),
            self.orientation,
        )


def test_recorded_world_pose_conversion_uses_parent_transform():
    gf = SimpleNamespace(
        Matrix4d=lambda _identity: _Matrix(),
        Quatd=lambda real, imaginary: (real, tuple(imaginary)),
        Vec3d=lambda *values: tuple(values),
    )
    parent_inverse = _Matrix(position=(-10.0, 5.0, -2.0))

    local = _world_pose_to_local_transform(
        gf,
        (0.4, -0.2, 0.31),
        (0.0, 0.0, 0.3826834324, 0.9238795325),
        parent_inverse,
    )

    assert local.position == (-9.6, 4.8, -1.69)
    assert local.orientation == (0.9238795325, (0.0, 0.0, 0.3826834324))
