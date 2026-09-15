import numpy as np
import pytest

import scene_builder.camera_extrinsic as camera_extrinsic
from scene_builder.camera_extrinsic import (
    author_camera_sensor_from_d455_color,
    d455_color_rigid_world_transform,
    find_d455_color_camera,
)
from scene_builder.d455_camera import (
    D455_ASSET_FALLBACK_URL,
    d455_asset_url,
    mount_d455,
)
from scene_builder.logical_camera import (
    CAMERA_SENSOR_CLIPPING_RANGE_M,
    CAMERA_SENSOR_FOCAL_LENGTH,
    create_logical_camera_sensor,
)


class _Invalid:
    def IsValid(self):
        return False


class _Prim:
    def __init__(self, name="", path="", valid=True, parent=None):
        self.name = name
        self.path = path
        self.valid = valid
        self.parent = parent
        self.custom_data = {}

    def IsValid(self):
        return self.valid

    def IsA(self, camera_type):
        return self.camera_type is camera_type

    def GetName(self):
        return self.name

    def GetPath(self):
        return self.path

    def GetParent(self):
        return self.parent

    def SetCustomDataByKey(self, key, value):
        self.custom_data[key] = value


class _Stage:
    def __init__(self, prims):
        self.prims = prims

    def GetPrimAtPath(self, path):
        return self.prims.get(path, _Invalid())


class _Usd:
    @staticmethod
    def PrimRange(root):
        return root.children


class _Matrix:
    def __init__(self, rows=None):
        self.rows = rows or [[0.0] * 4 for _ in range(4)]

    def __getitem__(self, index):
        return self.rows[index]

    def SetRow(self, index, value):
        self.rows[index] = list(value)

    def __mul__(self, other):
        return ("multiplied", self, other)

    def GetInverse(self):
        return "parent-inverse"


class _Gf:
    class Matrix4d(_Matrix):
        def __init__(self, value):
            super().__init__(
                [
                    [float(value if row == column else 0.0) for column in range(4)]
                    for row in range(4)
                ]
            )

    @staticmethod
    def Vec4d(*value):
        return value

    @staticmethod
    def Vec3d(*value):
        return value

    @staticmethod
    def Vec3f(*value):
        return value

    @staticmethod
    def Vec2f(*value):
        return value


def _usd_geom(composed_by_prim):
    class _Camera:
        pass

    class _XformCache:
        def GetLocalToWorldTransform(self, prim):
            return composed_by_prim[prim]

    return type("UsdGeom", (), {"Camera": _Camera, "XformCache": _XformCache})


def test_d455_color_lookup_requires_exactly_one_color_camera():
    usd_geom = _usd_geom({})
    color = _Prim("Camera_OmniVision_OV9782_Color", "/D455/Color")
    color.camera_type = usd_geom.Camera
    other = _Prim("Other", "/D455/Other")
    other.camera_type = usd_geom.Camera
    root = _Prim("D455", "/D455")
    root.children = [color, other]
    stage = _Stage({"/D455": root})

    assert find_d455_color_camera(stage, "/D455", usd=_Usd, usd_geom=usd_geom) is color
    root.children = [other]
    with pytest.raises(RuntimeError, match="Expected exactly one D455 Color camera"):
        find_d455_color_camera(stage, "/D455", usd=_Usd, usd_geom=usd_geom)


def test_accepted_d455_world_translation_and_proper_rotation_are_preserved():
    raw = _Matrix(
        [
            [1.000001, 0.0, 0.0, 0.0],
            [0.0, 0.999999, 0.0, 0.0],
            [0.0, 0.0, 1.0000005, 0.0],
            [0.2, -0.83, 1.725, 1.0],
        ]
    )
    usd_geom = _usd_geom({})
    color = _Prim("Camera_OmniVision_OV9782_Color", "/D455/Color")
    color.camera_type = usd_geom.Camera
    root = _Prim("D455", "/D455")
    root.children = [color]
    usd_geom = _usd_geom({color: raw})
    color.camera_type = usd_geom.Camera
    stage = _Stage({"/D455": root})

    rigid, color_path, evidence = d455_color_rigid_world_transform(
        stage, "/D455", gf=_Gf, usd=_Usd, usd_geom=usd_geom
    )

    assert color_path == "/D455/Color"
    assert rigid.rows[3] == [0.2, -0.83, 1.725, 1.0]
    assert np.linalg.det(
        np.array([row[:3] for row in rigid.rows[:3]])
    ) == pytest.approx(1.0)
    assert evidence["max_axis_norm_deviation"] < 1.0e-5


def test_d455_mount_owns_reference_mount_metadata_and_rigid_body_handling_only():
    d455_prim = _Prim(path="/D455")
    stage = _Stage({"/D455": d455_prim})
    calls = []
    reports = []

    class _Xformable:
        def __init__(self, prim):
            self.prim = prim

        def ClearXformOpOrder(self):
            calls.append("clear")

        def AddTranslateOp(self, precision):
            calls.append(("translate", precision))
            return type(
                "Op",
                (),
                {"Set": lambda self, value: calls.append(("translation", value))},
            )()

        def AddRotateXYZOp(self):
            calls.append("rotate")
            return type(
                "Op", (), {"Set": lambda self, value: calls.append(("rotation", value))}
            )()

    usd_geom = type(
        "UsdGeom",
        (),
        {
            "Xformable": _Xformable,
            "XformOp": type("Op", (), {"PrecisionDouble": "double"}),
        },
    )
    mount_d455(
        stage,
        "asset.usd",
        "/D455",
        [0.2, -0.83, 1.725],
        [0, 90, 0],
        add_reference=lambda **kwargs: calls.append(("reference", kwargs)),
        gf=_Gf,
        usd_geom=usd_geom,
        rigid_body_disabler=lambda prim: ["/D455/body"],
        report=reports.append,
    )

    assert calls == [
        (
            "reference",
            {"usd_path": "asset.usd", "prim_path": "/D455", "prim_type": "Xform"},
        ),
        "clear",
        ("translate", "double"),
        ("translation", (0.2, -0.83, 1.725)),
        "rotate",
        ("rotation", (0.0, 90.0, 0.0)),
    ]
    assert d455_prim.custom_data == {"sf_twin:role": "realsense_d455_visual"}
    assert d455_asset_url(None) == D455_ASSET_FALLBACK_URL
    assert (
        d455_asset_url("omniverse://assets")
        == "omniverse://assets/Isaac/Sensors/Intel/RealSense/rsd455.usd"
    )


def test_extrinsic_authoring_clears_then_authors_the_derived_local_transform(
    monkeypatch,
):
    world_transform = _Matrix(
        [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0.2, -0.83, 1.725, 1]]
    )
    parent = _Prim(path="/World/SF_Twin_Cell/Vision")
    camera_prim = _Prim(path="/World/SF_Twin_Cell/Vision/Camera_Sensor", parent=parent)
    calls = []

    monkeypatch.setattr(
        camera_extrinsic,
        "d455_color_rigid_world_transform",
        lambda *args, **kwargs: (world_transform, "/D455/Color", {"determinant": 1.0}),
    )

    class _XformCache:
        def GetLocalToWorldTransform(self, prim):
            assert prim is parent
            return _Matrix()

    class _Xformable:
        def __init__(self, prim):
            assert prim is camera_prim

        def ClearXformOpOrder(self):
            calls.append("clear")

        def AddTransformOp(self, precision):
            calls.append(("transform", precision))
            return type(
                "Op", (), {"Set": lambda self, value: calls.append(("set", value))}
            )()

    usd_geom = type(
        "UsdGeom",
        (),
        {
            "XformCache": _XformCache,
            "Xformable": _Xformable,
            "XformOp": type("Op", (), {"PrecisionDouble": "double"}),
        },
    )
    messages = []
    author_camera_sensor_from_d455_color(
        object(),
        camera_prim,
        "/D455",
        gf=_Gf,
        usd=_Usd,
        usd_geom=usd_geom,
        report=messages.append,
    )

    assert calls == [
        "clear",
        ("transform", "double"),
        ("set", ("multiplied", world_transform, "parent-inverse")),
    ]
    assert messages[0].endswith("/D455/Color")


def test_logical_camera_owns_intrinsics_metadata_and_extrinsic_invocation():
    camera_prim = _Prim(path="/World/SF_Twin_Cell/Vision/Camera_Sensor")
    focal = type(
        "Attribute", (), {"Set": lambda self, value: setattr(self, "value", value)}
    )()
    clipping = type(
        "Attribute", (), {"Set": lambda self, value: setattr(self, "value", value)}
    )()
    camera = type(
        "CameraInstance",
        (),
        {
            "GetPrim": lambda self: camera_prim,
            "GetFocalLengthAttr": lambda self: focal,
            "GetClippingRangeAttr": lambda self: clipping,
        },
    )()
    paths = []
    usd_geom = type(
        "UsdGeom",
        (),
        {
            "Camera": type(
                "Camera",
                (),
                {
                    "Define": staticmethod(
                        lambda stage, path: paths.append(path) or camera
                    )
                },
            )
        },
    )
    extrinsic_calls = []

    result = create_logical_camera_sensor(
        object(),
        "/World/SF_Twin_Cell",
        "/D455",
        gf=_Gf,
        usd_geom=usd_geom,
        extrinsic_author=lambda stage, prim, d455_path: extrinsic_calls.append(
            (stage, prim, d455_path)
        ),
    )

    assert result is camera
    assert paths == ["/World/SF_Twin_Cell/Vision/Camera_Sensor"]
    assert focal.value == CAMERA_SENSOR_FOCAL_LENGTH
    assert clipping.value == CAMERA_SENSOR_CLIPPING_RANGE_M
    assert camera_prim.custom_data == {
        "sf_twin:role": "amr_pickup_overhead_camera",
        "sf_twin:observes": "amr_tray_and_workpiece",
    }
    assert extrinsic_calls == [(extrinsic_calls[0][0], camera_prim, "/D455")]
