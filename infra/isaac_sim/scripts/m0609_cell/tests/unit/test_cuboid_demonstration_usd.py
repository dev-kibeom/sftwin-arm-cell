"""Demonstration transform regressions exercised with real OpenUSD matrices."""

import math

import pytest

pxr = pytest.importorskip("pxr")
from pxr import Gf, Usd, UsdGeom
from scene_builder.material_supply import MaterialSupplyProfile
from scene_builder.material_supply_adapter import (
    DELIVERY_ROOT,
    IsaacMaterialSupplyAdapter,
)


class _Sidecar:
    available = True


def _stub_physx_schema(monkeypatch):
    class _RigidBodyAPI:
        @staticmethod
        def Apply(_prim):
            return _RigidBodyAPI()

        @staticmethod
        def CreateDisableGravityAttr(_value):
            return None

    monkeypatch.setattr(
        pxr,
        "PhysxSchema",
        type("PhysxSchema", (), {"PhysxRigidBodyAPI": _RigidBodyAPI}),
        raising=False,
    )


def _adapter(stage, profile=None):
    profile = profile or MaterialSupplyProfile(
        material_type="RawPart",
        shape="box",
        dimensions_m=(0.04, 0.04, 0.04),
        mass_kg=0.1,
        receiving_x_m=(-0.1, 0.1),
        receiving_y_m=(-0.1, 0.1),
        spawn_height_above_shelf_m=0.1,
        spawn_interval_s=0.01,
        orientation_rpy_rad=((-1, 1), (-1, 1), (-1, 1)),
    )
    return IsaacMaterialSupplyAdapter(
        stage=stage,
        sidecar=_Sidecar(),
        profile=profile,
        shelf_origin_m=(0.0, 0.0, 0.0),
        delivery_deadline_ms=1000,
    )


def test_material_definition_shape_is_authored_on_generated_rawpart_root(monkeypatch):
    _stub_physx_schema(monkeypatch)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, DELIVERY_ROOT)
    profile = MaterialSupplyProfile.from_mapping(
        {
            "material_supply": {
                "material_type": "RawPart",
                "mass_kg": 0.1,
                "receiving_region_fraction": 0.5,
                "spawn_height_above_shelf_m": 0.1,
                "spawn_interval_s": 0.01,
                "position_distribution": "uniform",
                "orientation_distribution": "uniform_rpy",
                "orientation_rpy_rad": {
                    "roll": [-1, 1],
                    "pitch": [-1, 1],
                    "yaw": [-1, 1],
                },
            }
        },
        {
            "detector_profiles": {
                "RawPart": {
                    "material_id": "RawPart",
                    "support_region_dimensions_m": [0.6, 0.4],
                }
            }
        },
        {
            "RawPart": {
                "material_id": "RawPart",
                "shape": "box",
                "nominal_dimensions_m": [0.04, 0.04, 0.04],
            }
        },
    )
    adapter = _adapter(stage, profile=profile)

    part_path = adapter._create_part("d0" * 16, 0, (0, 0, 0.1, 0, 0, 0))
    root = stage.GetPrimAtPath(part_path)

    assert root.GetAttribute("sf_twin:shape").Get() == "box"
    assert root.GetAttribute("sf_twin:material_type").Get() == "RawPart"


def _define_transform(stage, path, translation, rotation):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    op = UsdGeom.Xformable(prim).AddTransformOp(
        precision=UsdGeom.XformOp.PrecisionDouble
    )
    transform = Gf.Matrix4d(1.0)
    transform.SetRotate(rotation)
    transform.SetTranslateOnly(Gf.Vec3d(*translation))
    op.Set(transform)


def _assert_pose(stage, path, expected_position, expected_orientation):
    prim = stage.GetPrimAtPath(path)
    world = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    position = world.Transform(Gf.Vec3d(0.0))
    orientation = world.ExtractRotationQuat().GetNormalized()
    expected = Gf.Quatd(
        expected_orientation[3], Gf.Vec3d(*expected_orientation[:3])
    ).GetNormalized()

    assert tuple(position) == pytest.approx(expected_position, abs=1e-9)
    # q and -q encode the same rotation; compare the normalized quaternion
    # with the closer sign so this checks orientation rather than convention.
    components = (orientation.GetReal(), *orientation.GetImaginary())
    expected_components = (expected.GetReal(), *expected.GetImaginary())
    if sum(a * b for a, b in zip(components, expected_components)) < 0:
        expected_components = tuple(-value for value in expected_components)
    assert components == pytest.approx(expected_components, abs=1e-9)


@pytest.mark.parametrize("transformed_parent", [False, True])
def test_demonstration_uses_world_pose_under_identity_or_transformed_ancestors(
    transformed_parent, monkeypatch
):
    _stub_physx_schema(monkeypatch)
    stage = Usd.Stage.CreateInMemory()
    for path in (
        "/World",
        "/World/SF_Twin_Cell",
        "/World/SF_Twin_Cell/MaterialSupply",
        DELIVERY_ROOT,
    ):
        UsdGeom.Xform.Define(stage, path)

    if transformed_parent:
        _define_transform(
            stage,
            "/World",
            (1.7, -0.8, 0.3),
            Gf.Quatd(math.cos(math.pi / 8), Gf.Vec3d(0, 0, math.sin(math.pi / 8))),
        )
        _define_transform(
            stage,
            f"{DELIVERY_ROOT}",
            (-0.25, 0.6, 0.15),
            Gf.Quatd(math.cos(math.pi / 12), Gf.Vec3d(math.sin(math.pi / 12), 0, 0)),
        )

    expected_position = (0.42, -0.17, 0.31)
    expected_orientation = (0.0, 0.0, math.sin(math.pi / 10), math.cos(math.pi / 10))
    adapter = IsaacMaterialSupplyAdapter(
        stage=stage,
        sidecar=_Sidecar(),
        profile=_adapter(stage).profile,
        shelf_origin_m=(0.0, 0.0, 0.0),
        delivery_deadline_ms=1000,
        timeline=type("Timeline", (), {"is_playing": lambda self: True})(),
    )
    adapter._create_demonstration_parts(
        [
            {
                "rawpart_id": "recorded-part",
                "prim_path": "/recorded/RawPart_0000",
                "position_xyz": expected_position,
                "orientation_xyzw": expected_orientation,
                "dimensions_m": (0.05, 0.04, 0.03),
                "mass_kg": 0.12,
            }
        ]
    )

    demonstration_prim = next(
        prim for prim in stage.Traverse() if prim.GetName() == "RawPart_0000"
    )
    _assert_pose(
        stage,
        str(demonstration_prim.GetPath()),
        expected_position,
        expected_orientation,
    )


def test_random_drop_part_creation_keeps_its_existing_local_pose(monkeypatch):
    _stub_physx_schema(monkeypatch)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, DELIVERY_ROOT)
    adapter = _adapter(stage)
    pose = (0.2, -0.1, 0.08, 0.0, 0.0, math.pi / 2)

    part_path = adapter._create_part("d0" * 16, 0, pose)

    _assert_pose(
        stage,
        part_path,
        pose[:3],
        (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
    )
    assert stage.GetPrimAtPath(part_path).GetAttribute("sf_twin:shape").Get() == "box"


def test_partial_demonstration_creation_removes_owned_root(monkeypatch):
    _stub_physx_schema(monkeypatch)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, DELIVERY_ROOT)
    timeline = type("Timeline", (), {"is_playing": lambda self: True})()
    base = _adapter(stage)
    adapter = IsaacMaterialSupplyAdapter(
        stage=stage,
        sidecar=_Sidecar(),
        profile=base.profile,
        shelf_origin_m=(0.0, 0.0, 0.0),
        delivery_deadline_ms=1000,
        timeline=timeline,
    )
    created = 0

    def fail_on_second(*_args, **_kwargs):
        nonlocal created
        created += 1
        if created == 2:
            raise RuntimeError("injected part creation failure")

    monkeypatch.setattr(adapter, "_create_part", fail_on_second)
    with pytest.raises(RuntimeError, match="injected part creation failure"):
        adapter.load_demonstration("side_by_side")
    assert not adapter._demonstration_loaded
    assert adapter._demonstration_paths == set()
    assert not any("Demonstration_" in str(prim.GetPath()) for prim in stage.Traverse())
