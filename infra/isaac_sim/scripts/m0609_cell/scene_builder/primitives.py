"""Low-level primitive authoring used by the M0609 scene composition root."""

import numpy as np

from scene_builder.materials import bind_material


def _pxr_modules():
    from pxr import Gf, UsdGeom

    return Gf, UsdGeom


def _cuboid_classes():
    try:
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    except ImportError:
        from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
    return FixedCuboid, DynamicCuboid


def fixed_box(
    stage,
    path,
    name,
    position,
    scale,
    color,
    material=None,
    *,
    fixed_cuboid_cls=None,
    material_binder=bind_material,
):
    """Author an Isaac fixed cuboid and optionally bind a material."""
    if fixed_cuboid_cls is None:
        fixed_cuboid_cls, _ = _cuboid_classes()
    obj = fixed_cuboid_cls(
        prim_path=path,
        name=name,
        position=np.array(position, dtype=float),
        scale=np.array(scale, dtype=float),
        color=np.array(color, dtype=float),
    )
    if material is not None:
        material_binder(stage, path, material)
    return obj


def dynamic_box(
    stage,
    path,
    name,
    position,
    scale,
    color,
    mass=0.4,
    material=None,
    *,
    dynamic_cuboid_cls=None,
    material_binder=bind_material,
):
    """Author an Isaac dynamic cuboid and optionally bind a material."""
    if dynamic_cuboid_cls is None:
        _, dynamic_cuboid_cls = _cuboid_classes()
    obj = dynamic_cuboid_cls(
        prim_path=path,
        name=name,
        position=np.array(position, dtype=float),
        scale=np.array(scale, dtype=float),
        color=np.array(color, dtype=float),
        mass=float(mass),
    )
    if material is not None:
        material_binder(stage, path, material)
    return obj


def create_cylinder(
    stage,
    path,
    radius,
    height,
    position,
    color,
    material=None,
    axis="Z",
    *,
    gf=None,
    usd_geom=None,
    material_binder=bind_material,
):
    """Author a translated USD cylinder with the builder's transform-op order."""
    if gf is None or usd_geom is None:
        default_gf, default_usd_geom = _pxr_modules()
        gf = gf or default_gf
        usd_geom = usd_geom or default_usd_geom
    cyl = usd_geom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(height))
    cyl.CreateAxisAttr(axis)
    xform = usd_geom.Xformable(cyl.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(precision=usd_geom.XformOp.PrecisionDouble).Set(
        gf.Vec3d(float(position[0]), float(position[1]), float(position[2]))
    )
    cyl.CreateDisplayColorAttr(
        [gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))]
    )
    if material is not None:
        material_binder(stage, path, material)
    return cyl


def create_sphere(
    stage,
    path,
    radius,
    position,
    color,
    material=None,
    *,
    gf=None,
    usd_geom=None,
    material_binder=bind_material,
):
    """Author a translated USD sphere with the builder's transform-op order."""
    if gf is None or usd_geom is None:
        default_gf, default_usd_geom = _pxr_modules()
        gf = gf or default_gf
        usd_geom = usd_geom or default_usd_geom
    sphere = usd_geom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(float(radius))
    xform = usd_geom.Xformable(sphere.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(precision=usd_geom.XformOp.PrecisionDouble).Set(
        gf.Vec3d(float(position[0]), float(position[1]), float(position[2]))
    )
    sphere.CreateDisplayColorAttr(
        [gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))]
    )
    if material is not None:
        material_binder(stage, path, material)
    return sphere
