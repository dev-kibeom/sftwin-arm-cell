import json
from pathlib import Path

import pytest

from pnp_validation.live_fixture import (
    AmrTopPlaneResolver,
    FixtureCommandWriter,
    FixtureResponseReader,
    FixtureRuntimeConsumer,
    candidate_identity,
    cube_geometry_from_transform,
    local_pose_provenance,
    sample_amr_top_candidate,
)


class FakePrim:
    def __init__(self, path, valid=True, world_transform=None):
        self.path = path
        self._valid = valid
        if world_transform is None:
            world_transform = {
                "translation": canonical_tray_geometry()["pose"]["position_m"]
            }
        self.world_transform = world_transform

    def IsValid(self):
        return self._valid


class FakeStage:
    def __init__(self, prims):
        self.prims = prims

    def GetPrimAtPath(self, path):
        return self.prims.get(path, FakePrim(path, valid=False))


def canonical_tray_geometry():
    root = Path(__file__).resolve().parents[6]
    manifest = json.loads(
        (root / "config/arm_cell/m0609_static_environment.json").read_text()
    )
    return {item["id"]: item for item in manifest["objects"]}["amr_tray"]


def test_command_writer_is_monotonic_and_response_correlates(tmp_path):
    command_path = tmp_path / "fixture_command.json"
    response_path = tmp_path / "fixture_response.json"
    writer = FixtureCommandWriter(command_path, run_id="run-1", fixture_id="fx-1")

    first = writer.write("spawn_fixture", {"candidate_id": "c-1"})
    second = writer.write("query_fixture", {})

    assert second["command_id"] > first["command_id"]
    assert not list(tmp_path.glob("*.tmp"))

    response_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "generation": second["generation"],
                "command_id": second["command_id"],
                "run_id": "run-1",
                "fixture_id": "fx-1",
                "status": "OK",
            }
        )
    )
    assert FixtureResponseReader(response_path).read_for(second)["status"] == "OK"


def test_consumer_rejects_stale_duplicate_and_foreign_commands(tmp_path):
    command_path = tmp_path / "fixture_command.json"
    response_path = tmp_path / "fixture_response.json"
    writer = FixtureCommandWriter(command_path, run_id="run-1", fixture_id="fx-1")
    command = writer.write("query_fixture", {})
    calls = []
    consumer = FixtureRuntimeConsumer(
        command_path,
        response_path,
        run_id="run-1",
        fixture_id="fx-1",
        handlers={
            "query_fixture": lambda payload: calls.append(payload) or {"ok": True}
        },
    )

    assert consumer.poll()["status"] == "OK"
    assert consumer.poll()["status"] == "OK"
    assert len(calls) == 1

    foreign = dict(command, run_id="other-run")
    command_path.write_text(json.dumps(foreign))
    assert consumer.poll()["status"] == "REJECTED"

    stale = dict(
        command,
        command_id=command["command_id"] - 1,
        generation=command["generation"] - 1,
    )
    command_path.write_text(json.dumps(stale))
    assert consumer.poll()["status"] == "REJECTED"


def test_consumer_restart_does_not_reexecute_last_command(tmp_path):
    command_path = tmp_path / "fixture_command.json"
    response_path = tmp_path / "fixture_response.json"
    writer = FixtureCommandWriter(command_path, run_id="run-1", fixture_id="fx-1")
    command = writer.write("query_fixture", {})
    calls = []
    kwargs = dict(
        command_path=command_path,
        response_path=response_path,
        run_id="run-1",
        fixture_id="fx-1",
        handlers={
            "query_fixture": lambda payload: calls.append(payload) or {"ok": True}
        },
    )
    assert FixtureRuntimeConsumer(**kwargs).poll()["status"] == "OK"
    assert FixtureRuntimeConsumer(**kwargs).poll()["status"] == "OK"
    assert len(calls) == 1


def test_response_reader_rejects_old_response_for_new_command(tmp_path):
    response_path = tmp_path / "fixture_response.json"
    response_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "generation": 1,
                "command_id": 1,
                "run_id": "run-1",
                "fixture_id": "fx-1",
                "status": "OK",
            }
        )
    )
    with pytest.raises(ValueError, match="command_id"):
        FixtureResponseReader(response_path).read_for(
            {"command_id": 2, "generation": 2, "run_id": "run-1", "fixture_id": "fx-1"}
        )


def test_amr_top_resolver_requires_exactly_one_canonical_prim():
    path = "/World/SF_Twin_Cell/AMR/Mockup/Tray"
    tray_center = canonical_tray_geometry()["pose"]["position_m"]
    result = AmrTopPlaneResolver(path).resolve(FakeStage({path: FakePrim(path)}))
    assert result.prim_path == path
    assert result.world_transform["translation"] == tray_center

    with pytest.raises(ValueError, match="missing"):
        AmrTopPlaneResolver(path).resolve(FakeStage({}))

    with pytest.raises(ValueError, match="multiple"):
        AmrTopPlaneResolver(path, matching_paths=[path, path + "/Duplicate"]).resolve(
            FakeStage({path: FakePrim(path)})
        )

    prim = FakePrim(path)
    prim.world_transform = None
    result = AmrTopPlaneResolver(path).resolve(
        FakeStage({path: prim}),
        world_transform_reader=lambda resolved: {"translation": tray_center},
    )
    assert result.world_transform["translation"] == tray_center


def test_frame_provenance_and_candidate_identity_are_deterministic():
    provenance = local_pose_provenance(
        support_surface_id="/World/SF_Twin_Cell/AMR/Mockup/Tray",
        support_surface_frame="amr_top",
        sampled_pose_amr_top={"x": 0.1, "y": -0.05, "z": 0.04, "yaw": -2.4},
        spawn_pose_world={"x": 0.3, "y": -0.9, "z": 0.72, "yaw": -2.4},
    )
    assert provenance["spawn_pose_world"]["frame_id"] == "world"
    assert provenance["sampled_pose_amr_top"]["frame_id"] == "amr_top"
    assert candidate_identity(
        seed=17, index=3, local_pose=provenance["sampled_pose_amr_top"]
    ) == candidate_identity(
        seed=17, index=3, local_pose=provenance["sampled_pose_amr_top"]
    )


def test_amr_candidate_sampling_preserves_seed_and_clearance_contract():
    tray = canonical_tray_geometry()
    tray_center = tray["pose"]["position_m"]
    tray_dimensions = tray["geometry"]["dimensions_m"]
    top_surface_z = tray_center[2] + tray_dimensions[2] / 2
    first = sample_amr_top_candidate(
        seed=17,
        index=2,
        usable_dimensions=(0.50, 0.30),
        fixture_dimensions=(0.07, 0.07, 0.08),
        top_surface_z=top_surface_z,
        clearance_m=0.001,
    )
    second = sample_amr_top_candidate(
        seed=17,
        index=2,
        usable_dimensions=(0.50, 0.30),
        fixture_dimensions=(0.07, 0.07, 0.08),
        top_surface_z=top_surface_z,
        clearance_m=0.001,
    )
    assert first == second
    assert first["pose"]["frame_id"] == "amr_top"
    assert first["pose"]["z"] == pytest.approx(top_surface_z + 0.041)


def test_cube_geometry_applies_scaled_local_cube_transform_once():
    tray = canonical_tray_geometry()
    center = tray["pose"]["position_m"]
    dimensions = tray["geometry"]["dimensions_m"]
    geometry = cube_geometry_from_transform(
        [
            [dimensions[0], 0.0, 0.0, center[0]],
            [0.0, dimensions[1], 0.0, center[1]],
            [0.0, 0.0, dimensions[2], center[2]],
            [0.0, 0.0, 0.0, 1.0],
        ],
        size=1.0,
    )
    assert geometry["center"] == pytest.approx(center)
    assert geometry["dimensions_m"] == pytest.approx(dimensions)
    assert geometry["aabb_min"] == pytest.approx(
        [center[index] - dimensions[index] / 2 for index in range(3)]
    )
    assert geometry["aabb_max"] == pytest.approx(
        [center[index] + dimensions[index] / 2 for index in range(3)]
    )
    assert geometry["top_z_m"] == pytest.approx(center[2] + dimensions[2] / 2)


def test_usd_row_vector_transform_is_transposed_at_geometry_boundary():
    from pnp_validation.run_validation import IsaacFixtureRuntime

    tray = canonical_tray_geometry()
    center = tray["pose"]["position_m"]
    dimensions = tray["geometry"]["dimensions_m"]
    usd_rows = [
        [dimensions[0], 0.0, 0.0, 0.0],
        [0.0, dimensions[1], 0.0, 0.0],
        [0.0, 0.0, dimensions[2], 0.0],
        [center[0], center[1], center[2], 1.0],
    ]

    column_matrix = IsaacFixtureRuntime._matrix_rows_for_geometry(usd_rows)
    geometry = cube_geometry_from_transform(column_matrix)

    assert geometry["center"] == pytest.approx(center)
    assert geometry["top_z_m"] == pytest.approx(center[2] + dimensions[2] / 2)


def test_amr_local_spawn_z_converts_fixture_height_through_support_scale_once():
    tray = canonical_tray_geometry()
    tray_center = tray["pose"]["position_m"]
    tray_dimensions = tray["geometry"]["dimensions_m"]
    fixture_dimensions = (0.07, 0.07, 0.08)
    clearance_m = 0.001
    candidate = sample_amr_top_candidate(
        seed=17,
        index=0,
        usable_dimensions=(1.0, 1.0),
        usable_bounds_local={"x_min": -0.4, "x_max": 0.4, "y_min": -0.4, "y_max": 0.4},
        fixture_dimensions=fixture_dimensions,
        sampling_fixture_dimensions=(
            fixture_dimensions[0] / tray_dimensions[0],
            fixture_dimensions[1] / tray_dimensions[1],
            fixture_dimensions[2] / tray_dimensions[2],
        ),
        top_surface_z=0.5,
        clearance_m=clearance_m / tray_dimensions[2],
    )
    expected_local_z = (
        0.5
        + fixture_dimensions[2] / tray_dimensions[2] / 2
        + clearance_m / tray_dimensions[2]
    )
    assert candidate["pose"]["z"] == pytest.approx(expected_local_z)
    world_z = tray_center[2] + tray_dimensions[2] * candidate["pose"]["z"]
    expected_world_z = (
        tray_center[2]
        + tray_dimensions[2] / 2
        + fixture_dimensions[2] / 2
        + clearance_m
    )
    assert world_z == pytest.approx(expected_world_z)


def test_amr_snapshot_assertion_rejects_scale_squared_or_wrong_top():
    from pnp_validation.run_validation import IsaacFixtureRuntime

    tray = canonical_tray_geometry()
    center = tray["pose"]["position_m"]
    dimensions = tray["geometry"]["dimensions_m"]
    support = {
        "world_center_m": center,
        "world_dimensions_m": dimensions,
        "world_top_z_m": center[2] + dimensions[2] / 2,
    }
    assert IsaacFixtureRuntime.assert_amr_top_plane_snapshot(support)
    support["world_dimensions_m"] = [dimension**2 for dimension in dimensions]
    with pytest.raises(RuntimeError, match="dimensions"):
        IsaacFixtureRuntime.assert_amr_top_plane_snapshot(support)
