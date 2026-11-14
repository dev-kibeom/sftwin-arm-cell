import json
import math

import pytest

from camera_tooling.renderer_depth_target_spec import (
    CANONICAL_FRAME,
    CANONICAL_TARGETS,
    TARGET_DEPTHS_M,
)
from camera_tooling import renderer_depth_targets as authoring


APPROVED_XYZ = (
    (-0.2739429677526156, -0.12114609330892565, 0.80),
    (0.3196456043049693, -0.18158271418263516, 1.10),
    (0.5090045879905422, 0.0, 1.40),
)


class FakePrim:
    def __init__(self, path, kind="Xform"):
        self.path = path
        self.kind = kind
        self.children = {}
        self.matrix = None
        self.scale = None
        self.valid = True

    def IsValid(self):
        return self.valid

    def GetChildren(self):
        return list(self.children.values())

    def GetName(self):
        return self.path.rsplit("/", 1)[-1]

    def GetPath(self):
        return self.path


class FakeStage:
    def __init__(self):
        self.prims = {}

    def DefinePrim(self, path, kind="Xform"):
        path = str(path)
        if path in self.prims:
            prim = self.prims[path]
            prim.kind = kind
            return prim
        parent_path = path.rsplit("/", 1)[0]
        if parent_path and parent_path != path:
            parent = self.DefinePrim(parent_path)
            prim = FakePrim(path, kind)
            parent.children[prim.GetName()] = prim
        else:
            prim = FakePrim(path, kind)
        self.prims[path] = prim
        return prim

    def GetPrimAtPath(self, path):
        return self.prims.get(str(path), FakePrim(str(path), "Invalid"))

    def RemovePrim(self, path):
        path = str(path)
        for candidate in list(self.prims):
            if candidate == path or candidate.startswith(path + "/"):
                del self.prims[candidate]
        parent_path = path.rsplit("/", 1)[0]
        parent = self.prims.get(parent_path)
        if parent:
            parent.children.pop(path.rsplit("/", 1)[-1], None)


def _install_fake_pxr(monkeypatch):
    class Matrix4d:
        def __init__(self, _identity):
            self.rows = [[0.0] * 4 for _ in range(4)]

        def SetRow(self, index, values):
            self.rows[index] = list(values)

        def __getitem__(self, index):
            return self.rows[index]

    class Xform:
        @staticmethod
        def Define(stage, path):
            prim = stage.DefinePrim(path, "Xform")
            return XformWrapper(prim)

    class XformWrapper:
        def __init__(self, prim):
            self.prim = prim

        def AddTransformOp(self, **_kwargs):
            return Op(self.prim, "matrix")

    class Xformable:
        def __init__(self, prim):
            self.prim = prim

        def AddScaleOp(self):
            return Op(self.prim, "scale")

    class Op:
        def __init__(self, prim, field):
            self.prim = prim
            self.field = field

        def Set(self, value):
            setattr(self.prim, self.field, value)

    class Cube:
        @staticmethod
        def Define(stage, path):
            return CubeWrapper(stage.DefinePrim(path, "Cube"))

    class CubeWrapper:
        def __init__(self, prim):
            self.prim = prim

        def GetSizeAttr(self):
            return Op(self.prim, "size")

        def GetPrim(self):
            return self.prim

    class Material:
        @staticmethod
        def Define(stage, path):
            return MaterialWrapper(stage.DefinePrim(path, "Material"))

    class MaterialWrapper:
        def __init__(self, prim):
            self.prim = prim

        def CreateSurfaceOutput(self):
            return Output()

    class Shader:
        @staticmethod
        def Define(stage, path):
            return ShaderWrapper(stage.DefinePrim(path, "Shader"))

    class ShaderWrapper:
        def __init__(self, prim):
            self.prim = prim

        def CreateIdAttr(self, _value):
            return None

        def CreateInput(self, _name, _type):
            return Op(self.prim, "color")

        def ConnectableAPI(self):
            return self

    class Output:
        def ConnectToSource(self, *_args):
            return None

    class MaterialBindingAPI:
        def __init__(self, _prim):
            pass

        def Bind(self, _material):
            return None

    class XformCache:
        def GetLocalToWorldTransform(self, prim):
            return prim.matrix

    from types import SimpleNamespace

    pxr = SimpleNamespace(
        Gf=SimpleNamespace(
            Matrix4d=Matrix4d,
            Vec4d=lambda *values: tuple(values),
            Vec3f=lambda *values: tuple(values),
        ),
        Sdf=SimpleNamespace(ValueTypeNames=SimpleNamespace(Color3f="Color3f")),
        UsdGeom=SimpleNamespace(
            Xform=Xform,
            Xformable=Xformable,
            Cube=Cube,
            XformOp=SimpleNamespace(PrecisionDouble="double"),
            XformCache=XformCache,
        ),
        UsdShade=SimpleNamespace(
            Material=Material,
            Shader=Shader,
            MaterialBindingAPI=MaterialBindingAPI,
        ),
    )
    monkeypatch.setitem(__import__("sys").modules, "pxr", pxr)


def snapshot():
    return {
        "camera_prim": "/World/Cell/Camera",
        "world_to_optical": {
            "parent_frame": "world",
            "child_frame": CANONICAL_FRAME,
            "translation_m": [1.0, 2.0, 3.0],
            "quaternion_xyzw": [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)],
        },
    }


def test_b2_u01_canonical_spec_matches_approved_xyz():
    assert len(CANONICAL_TARGETS) == 3
    assert tuple(target.target_id for target in CANONICAL_TARGETS) == (
        "z080",
        "z110",
        "z140",
    )
    assert (
        tuple(target.front_optical_xyz_m for target in CANONICAL_TARGETS)
        == APPROVED_XYZ
    )
    assert all(target.frame_id == CANONICAL_FRAME for target in CANONICAL_TARGETS)


def test_b2_u02_z_has_one_canonical_source_of_truth():
    assert TARGET_DEPTHS_M == tuple(xyz[2] for xyz in APPROVED_XYZ)
    assert all(
        "expected_z" not in target.__dataclass_fields__ for target in CANONICAL_TARGETS
    )


def test_b2_u03_optical_target_is_placed_from_current_snapshot():
    placements = authoring.target_placements(snapshot())
    # A 90-degree optical-to-world rotation maps (x, y, z) to (-y, x, z).
    first = placements[0]
    assert first["front_world_m"] == pytest.approx(
        (1.1211460933089256, 1.7260570322473844, 3.8)
    )
    assert first["center_world_m"] == pytest.approx(
        (1.1211460933089256, 1.7260570322473844, 3.802)
    )


def test_b2_u04_targets_are_authored_only_in_acceptance_namespace(monkeypatch):
    _install_fake_pxr(monkeypatch)
    stage = FakeStage()
    report = authoring.author_targets(stage, snapshot())
    assert report["target_paths"] == [
        f"{authoring.ROOT}/z080",
        f"{authoring.ROOT}/z110",
        f"{authoring.ROOT}/z140",
    ]
    assert all(path.startswith(authoring.ROOT + "/") for path in report["target_paths"])


def test_b2_u05_authoring_replaces_stale_namespace_idempotently(monkeypatch):
    _install_fake_pxr(monkeypatch)
    stage = FakeStage()
    stage.DefinePrim("/World/SF_Twin_Acceptance/RendererDepth/stale")
    stage.DefinePrim("/World/Production/Keep")
    first = authoring.author_targets(stage, snapshot())
    first_paths = sorted(
        path for path in stage.prims if path.startswith(authoring.ROOT + "/")
    )
    second = authoring.author_targets(stage, snapshot())
    second_paths = sorted(
        path for path in stage.prims if path.startswith(authoring.ROOT + "/")
    )
    assert first["target_ids"] == second["target_ids"] == ["z080", "z110", "z140"]
    assert first_paths == second_paths
    assert not any(path.endswith("/stale") for path in second_paths)
    assert "/World/Production/Keep" in stage.prims


def test_b2_u06_authoring_needs_only_snapshot_not_target_plan(tmp_path, monkeypatch):
    _install_fake_pxr(monkeypatch)
    snapshot_path = tmp_path / "camera_snapshot.json"
    snapshot_path.write_text(json.dumps(snapshot()))
    stage = FakeStage()
    report = authoring.create(snapshot_path, stage=stage)
    assert report["target_ids"] == ["z080", "z110", "z140"]


def test_b2_u07_authoring_does_not_require_or_write_manifest(tmp_path, monkeypatch):
    _install_fake_pxr(monkeypatch)
    snapshot_path = tmp_path / "camera_snapshot.json"
    snapshot_path.write_text(json.dumps(snapshot()))
    stage = FakeStage()
    authoring.create(snapshot_path, stage=stage)
    assert not (tmp_path / "depth_calibration_manifest.json").exists()
    assert not list(tmp_path.glob("*manifest*"))
