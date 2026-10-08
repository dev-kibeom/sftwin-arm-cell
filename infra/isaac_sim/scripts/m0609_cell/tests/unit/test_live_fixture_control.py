import json

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
        self.world_transform = world_transform or {"translation": [0.2, -0.85, 0.64]}

    def IsValid(self):
        return self._valid


class FakeStage:
    def __init__(self, prims):
        self.prims = prims

    def GetPrimAtPath(self, path):
        return self.prims.get(path, FakePrim(path, valid=False))


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
    result = AmrTopPlaneResolver(path).resolve(FakeStage({path: FakePrim(path)}))
    assert result.prim_path == path
    assert result.world_transform["translation"] == [0.2, -0.85, 0.64]

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
        world_transform_reader=lambda resolved: {"translation": [0.2, -0.85, 0.64]},
    )
    assert result.world_transform["translation"] == [0.2, -0.85, 0.64]


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
    first = sample_amr_top_candidate(
        seed=17,
        index=2,
        usable_dimensions=(0.50, 0.30),
        fixture_dimensions=(0.07, 0.07, 0.08),
        top_surface_z=0.6625,
        clearance_m=0.001,
    )
    second = sample_amr_top_candidate(
        seed=17,
        index=2,
        usable_dimensions=(0.50, 0.30),
        fixture_dimensions=(0.07, 0.07, 0.08),
        top_surface_z=0.6625,
        clearance_m=0.001,
    )
    assert first == second
    assert first["pose"]["frame_id"] == "amr_top"
    assert first["pose"]["z"] == pytest.approx(0.7035)


def test_cube_geometry_applies_scaled_local_cube_transform_once():
    geometry = cube_geometry_from_transform(
        [
            [0.66, 0.0, 0.0, 0.20],
            [0.0, 0.46, 0.0, -0.85],
            [0.0, 0.0, 0.045, 0.64],
            [0.0, 0.0, 0.0, 1.0],
        ],
        size=1.0,
    )
    assert geometry["center"] == pytest.approx([0.20, -0.85, 0.64])
    assert geometry["dimensions_m"] == pytest.approx([0.66, 0.46, 0.045])
    assert geometry["aabb_min"] == pytest.approx([-0.13, -1.08, 0.6175])
    assert geometry["aabb_max"] == pytest.approx([0.53, -0.62, 0.6625])
    assert geometry["top_z_m"] == pytest.approx(0.6625)


def test_usd_row_vector_transform_is_transposed_at_geometry_boundary():
    from pnp_validation.run_validation import IsaacFixtureRuntime

    usd_rows = [
        [0.66, 0.0, 0.0, 0.0],
        [0.0, 0.46, 0.0, 0.0],
        [0.0, 0.0, 0.045, 0.0],
        [0.20, -0.85, 0.64, 1.0],
    ]

    column_matrix = IsaacFixtureRuntime._matrix_rows_for_geometry(usd_rows)
    geometry = cube_geometry_from_transform(column_matrix)

    assert geometry["center"] == pytest.approx([0.20, -0.85, 0.64])
    assert geometry["top_z_m"] == pytest.approx(0.6625)


def test_amr_local_spawn_z_converts_fixture_height_through_support_scale_once():
    candidate = sample_amr_top_candidate(
        seed=17,
        index=0,
        usable_dimensions=(1.0, 1.0),
        usable_bounds_local={"x_min": -0.4, "x_max": 0.4, "y_min": -0.4, "y_max": 0.4},
        fixture_dimensions=(0.07, 0.07, 0.08),
        sampling_fixture_dimensions=(0.07 / 0.66, 0.07 / 0.46, 0.08 / 0.045),
        top_surface_z=0.5,
        clearance_m=0.001 / 0.045,
    )
    assert candidate["pose"]["z"] == pytest.approx(1.4111111111, abs=1e-9)
    world_z = 0.64 + 0.045 * candidate["pose"]["z"]
    assert world_z == pytest.approx(0.7025 + 0.001)


def test_amr_snapshot_assertion_rejects_scale_squared_or_wrong_top():
    from pnp_validation.run_validation import IsaacFixtureRuntime

    support = {
        "world_center_m": [0.20, -0.85, 0.64],
        "world_dimensions_m": [0.66, 0.46, 0.045],
        "world_top_z_m": 0.6625,
    }
    assert IsaacFixtureRuntime.assert_amr_top_plane_snapshot(support)
    support["world_dimensions_m"] = [0.4356, 0.2116, 0.002025]
    with pytest.raises(RuntimeError, match="dimensions"):
        IsaacFixtureRuntime.assert_amr_top_plane_snapshot(support)
