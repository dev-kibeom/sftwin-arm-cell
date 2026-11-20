"""Author the collidable geometry and dynamic body of one supplied workpiece."""


def author_cuboid_body(stage, prim, dimensions, mass, display_color_rgb):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    shape = UsdGeom.Cube.Define(stage, prim.GetPath().AppendChild("collision"))
    shape.CreateSizeAttr(1.0)
    shape_xform = UsdGeom.Xformable(shape.GetPrim())
    shape_xform.ClearXformOpOrder()
    shape_xform.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*dimensions)
    )

    color = Gf.Vec3f(*display_color_rgb)
    UsdGeom.Gprim(shape.GetPrim()).CreateDisplayColorPrimvar().Set([color])
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(mass)
    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    physx_body.CreateDisableGravityAttr(False)
    return shape
