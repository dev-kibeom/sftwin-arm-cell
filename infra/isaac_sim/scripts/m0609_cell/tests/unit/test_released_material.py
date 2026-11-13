from types import SimpleNamespace

import pytest

from gripper_runtime import released_material


def handler(*, fail_gravity=False, fail_observe=False, fail_submit=False):
    events, records = [], []

    def enable(path):
        events.append(("gravity", path))
        if fail_gravity:
            raise RuntimeError("gravity unavailable")

    def observe(path):
        events.append(("observe", path))
        if fail_observe:
            raise RuntimeError("pose unavailable")
        return {"source": "test_rigid_body", "position_m": (0.1, 0.2, 0.3)}

    def submit(record):
        if fail_submit:
            raise RuntimeError("writer unavailable")
        records.append(record)

    return (
        released_material.ReleasedMaterialHandler(enable, observe, submit),
        events,
        records,
    )


def test_detached_material_enables_gravity_before_observing_after_physics_advances():
    instance, events, records = handler()
    instance.on_detached("/World/Plate", 7)
    assert events == [("gravity", "/World/Plate")]
    instance.on_physics_step(0.0)
    assert len(events) == 1
    for _ in range(4):
        instance.on_physics_step(1 / 60)
    instance.on_physics_step(1 / 60)
    assert events == [("gravity", "/World/Plate")] + [("observe", "/World/Plate")] * 4
    assert len(records) == 1
    assert records[0]["prim_path"] == "/World/Plate"
    assert records[0]["action_graph_command_generation"] == 7
    assert records[0]["gravity_activation"]["status"] == "enabled"
    assert records[0]["samples"][0]["elapsed_physics_s"] == pytest.approx(1 / 60)
    assert records[0]["samples"][0]["body"]["position_m"] == (0.1, 0.2, 0.3)


@pytest.mark.parametrize("failure", ["gravity", "observe", "submit"])
def test_release_postprocessing_failure_is_diagnostic_only(failure):
    instance, events, records = handler(**{f"fail_{failure}": True})
    instance.on_detached("/World/Cube", None)
    for _ in range(4):
        instance.on_physics_step(0.01)
    if failure == "gravity":
        assert records[0]["gravity_activation"]["status"] == "failed"
        assert "gravity unavailable" in records[0]["gravity_activation"]["reason"]
    elif failure == "observe":
        assert records[0]["samples"][0]["body"]["status"] == "unavailable"
    else:
        assert records == []


def test_stop_cancels_pending_post_release_observations_without_physics_mutation():
    instance, events, records = handler()
    instance.on_detached("/World/Plate", 7)
    instance.reset()
    instance.on_physics_step(0.01)
    assert events == [("gravity", "/World/Plate")]
    assert records[0]["completion"] == "interrupted"


def test_consecutive_releases_preserve_each_target_and_bound_observation_work():
    instance, events, records = handler()
    instance.on_detached("/World/Plate", 7)
    instance.on_detached("/World/Billet", 8)
    for _ in range(4):
        instance.on_physics_step(0.01)
    assert [r["prim_path"] for r in records] == ["/World/Plate", "/World/Billet"]
    assert records[0]["completion"] == "superseded"
    assert records[1]["completion"] == "observed"
    assert ("observe", "/World/Plate") not in events


@pytest.mark.parametrize(
    "valid,rigid_body", [(True, True), (False, True), (True, False)]
)
def test_common_gravity_activation_requires_target_rigid_body_and_canonical_schema(
    monkeypatch, valid, rigid_body
):
    import sys

    values = []
    prim = SimpleNamespace(IsValid=lambda: valid, HasAPI=lambda api: rigid_body)
    stage = SimpleNamespace(
        GetPrimAtPath=lambda path: prim if path == "/World/Plate" else None
    )
    monkeypatch.setitem(
        sys.modules,
        "pxr",
        SimpleNamespace(
            UsdPhysics=SimpleNamespace(RigidBodyAPI=object()),
            PhysxSchema=SimpleNamespace(
                PhysxRigidBodyAPI=SimpleNamespace(
                    Apply=lambda target: SimpleNamespace(
                        CreateDisableGravityAttr=lambda: SimpleNamespace(
                            Set=values.append
                        )
                    )
                )
            ),
        ),
    )
    if valid and rigid_body:
        released_material.enable_material_gravity(stage, "/World/Plate")
        assert values == [False]
    else:
        with pytest.raises(RuntimeError, match="detached rigid body is unavailable"):
            released_material.enable_material_gravity(stage, "/World/Plate")
        assert values == []
