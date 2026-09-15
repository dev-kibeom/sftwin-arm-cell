import numpy as np

from scene_builder.materials import create_preview_material
from scene_builder.primitives import (
    create_cylinder,
    create_sphere,
    dynamic_box,
    fixed_box,
)
from scene_builder.static_environment import author_static_box
from scene_builder.usd_authoring import (
    ensure_xform,
    set_custom_data,
    set_vec3_attribute,
)


class _Prim:
    def __init__(self, valid=True):
        self.valid = valid
        self.custom_data = {}
        self.attributes = {}

    def IsValid(self):
        return self.valid

    def SetCustomDataByKey(self, key, value):
        self.custom_data[key] = value

    def GetAttribute(self, name):
        return self.attributes.get(name, _Prim(False))

    def CreateAttribute(self, name, type_name, custom):
        attribute = _Attribute(type_name, custom)
        self.attributes[name] = attribute
        return attribute


class _Attribute:
    def __init__(self, type_name=None, custom=None):
        self.type_name = type_name
        self.custom = custom
        self.value = None

    def IsValid(self):
        return True

    def Set(self, value):
        self.value = value


class _Stage:
    def __init__(self, prims):
        self.prims = prims

    def GetPrimAtPath(self, path):
        return self.prims.get(path, _Prim(False))


class _Gf:
    @staticmethod
    def Vec3d(*value):
        return value

    @staticmethod
    def Vec3f(*value):
        return value


class _Sdf:
    class ValueTypeNames:
        Double3 = "Double3"
        Color3f = "Color3f"
        Float = "Float"


def test_usd_authoring_preserves_invalid_prim_warning_and_metadata_contract():
    valid = _Prim()
    stage = _Stage({"/Valid": valid})
    warnings = []

    set_custom_data(stage, "/Valid", "sf_twin:role", "fixture")
    attribute = set_vec3_attribute(
        stage,
        "/Valid",
        "sf_twin:position",
        [1, 2, 3],
        gf=_Gf,
        sdf=_Sdf,
        warn=warnings.append,
    )
    missing = set_vec3_attribute(
        stage,
        "/Missing",
        "sf_twin:position",
        [1, 2, 3],
        gf=_Gf,
        sdf=_Sdf,
        warn=warnings.append,
    )

    assert valid.custom_data == {"sf_twin:role": "fixture"}
    assert attribute.type_name == "Double3"
    assert attribute.custom is True
    assert attribute.value == (1.0, 2.0, 3.0)
    assert missing is None
    assert warnings == [
        ">>> [WARN] Cannot create vec3 attribute; invalid prim: /Missing"
    ]


def test_ensure_xform_defines_only_missing_prim():
    existing = _Prim()
    defined = _Prim()
    stage = _Stage({"/Existing": existing})

    class _Xform:
        @staticmethod
        def Define(received_stage, path):
            assert received_stage is stage
            assert path == "/Missing"
            return type("Defined", (), {"GetPrim": lambda self: defined})()

    usd_geom = type("UsdGeom", (), {"Xform": _Xform})
    assert ensure_xform(stage, "/Existing", usd_geom=usd_geom) is existing
    assert ensure_xform(stage, "/Missing", usd_geom=usd_geom) is defined


def test_manifest_static_box_forwards_canonical_geometry_unchanged():
    geometry = {
        "factory_floor": {
            "pose": {"position_m": [1.0, 2.0, 3.0]},
            "geometry": {"dimensions_m": [4.0, 5.0, 6.0]},
        }
    }
    received = []

    result = author_static_box(
        geometry,
        lambda *args: received.append(args) or "box",
        "factory_floor",
        "/World/Floor",
        "Floor",
        [0.1, 0.2, 0.3],
        "floor-material",
    )

    assert result == "box"
    assert received == [
        (
            "/World/Floor",
            "Floor",
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [0.1, 0.2, 0.3],
            "floor-material",
        )
    ]


def test_cuboid_helpers_preserve_numeric_inputs_and_material_binding():
    created = []
    bindings = []

    class _Cuboid:
        def __init__(self, **kwargs):
            created.append(kwargs)

    stage = object()
    assert (
        fixed_box(
            stage,
            "/Fixed",
            "fixed",
            [1, 2, 3],
            [4, 5, 6],
            [0.1, 0.2, 0.3],
            "mat",
            fixed_cuboid_cls=_Cuboid,
            material_binder=lambda *args: bindings.append(args),
        ).__class__
        is _Cuboid
    )
    dynamic_box(
        stage,
        "/Dynamic",
        "dynamic",
        [1, 2, 3],
        [4, 5, 6],
        [0.1, 0.2, 0.3],
        0.7,
        "mat",
        dynamic_cuboid_cls=_Cuboid,
        material_binder=lambda *args: bindings.append(args),
    )

    assert np.array_equal(created[0]["position"], np.array([1.0, 2.0, 3.0]))
    assert created[1]["mass"] == 0.7
    assert bindings == [(stage, "/Fixed", "mat"), (stage, "/Dynamic", "mat")]


def test_cylinder_and_sphere_keep_clear_then_double_precision_translate_order():
    calls = []

    class _Shape:
        def __init__(self):
            self.radius = None
            self.height = None
            self.axis = None
            self.color = None
            self.prim = object()

        def CreateRadiusAttr(self, value):
            self.radius = value

        def CreateHeightAttr(self, value):
            self.height = value

        def CreateAxisAttr(self, value):
            self.axis = value

        def GetPrim(self):
            return self.prim

        def CreateDisplayColorAttr(self, value):
            self.color = value

    cylinder = _Shape()
    sphere = _Shape()

    class _Xformable:
        def __init__(self, prim):
            self.prim = prim

        def ClearXformOpOrder(self):
            calls.append((self.prim, "clear"))

        def AddTranslateOp(self, precision):
            calls.append((self.prim, "translate", precision))
            return _Attribute()

    usd_geom = type(
        "UsdGeom",
        (),
        {
            "Cylinder": type(
                "Cylinder", (), {"Define": staticmethod(lambda stage, path: cylinder)}
            ),
            "Sphere": type(
                "Sphere", (), {"Define": staticmethod(lambda stage, path: sphere)}
            ),
            "Xformable": _Xformable,
            "XformOp": type("XformOp", (), {"PrecisionDouble": "double"}),
        },
    )

    create_cylinder(
        object(),
        "/Cylinder",
        0.2,
        0.3,
        [1, 2, 3],
        [0.1, 0.2, 0.3],
        gf=_Gf,
        usd_geom=usd_geom,
    )
    create_sphere(
        object(), "/Sphere", 0.4, [4, 5, 6], [0.4, 0.5, 0.6], gf=_Gf, usd_geom=usd_geom
    )

    assert calls == [
        (cylinder.prim, "clear"),
        (cylinder.prim, "translate", "double"),
        (sphere.prim, "clear"),
        (sphere.prim, "translate", "double"),
    ]
    assert cylinder.radius == 0.2 and cylinder.height == 0.3 and cylinder.axis == "Z"
    assert sphere.radius == 0.4


def test_preview_material_authors_original_preview_surface_inputs():
    inputs = {}

    class _Output:
        def ConnectToSource(self, source, output):
            self.connection = (source, output)

    class _Material:
        def CreateSurfaceOutput(self):
            self.output = _Output()
            return self.output

    class _Shader:
        def CreateIdAttr(self, value):
            self.identifier = value

        def CreateInput(self, name, type_name):
            attribute = _Attribute(type_name)
            inputs[name] = attribute
            return attribute

        def ConnectableAPI(self):
            return "shader-api"

    material = _Material()
    shader = _Shader()
    usd_shade = type(
        "UsdShade",
        (),
        {
            "Material": type(
                "Material", (), {"Define": staticmethod(lambda stage, path: material)}
            ),
            "Shader": type(
                "Shader", (), {"Define": staticmethod(lambda stage, path: shader)}
            ),
        },
    )

    result = create_preview_material(
        object(),
        "/World/Cell",
        "Steel",
        [0.1, 0.2, 0.3],
        0.4,
        0.5,
        0.6,
        gf=_Gf,
        sdf=_Sdf,
        usd_shade=usd_shade,
    )

    assert result is material
    assert shader.identifier == "UsdPreviewSurface"
    assert inputs["diffuseColor"].value == (0.1, 0.2, 0.3)
    assert inputs["roughness"].value == 0.4
    assert inputs["metallic"].value == 0.5
    assert inputs["opacity"].value == 0.6
    assert material.output.connection == ("shader-api", "surface")
