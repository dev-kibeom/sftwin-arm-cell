"""Manual-only gripper attachment probe; never imported by production runtime."""


def run_probe():
    """Create the legacy test workpiece and expose its unchanged probe state."""
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics
    from gripper_runtime.grasp_policy import GraspConfig

    stage = omni.usd.get_context().get_stage()
    TARGET_PATH = "/World/TestWorkpiece"
    prim = stage.GetPrimAtPath(TARGET_PATH)
    if not prim.IsValid():
        cube = UsdGeom.Cube.Define(stage, TARGET_PATH)
        cube.CreateSizeAttr(0.04)
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.0))
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(cube.GetPrim())
        mass.CreateMassAttr().Set(0.1)
        print(f">>> [INFO] Created test workpiece: {TARGET_PATH}")

    GRASP_CONFIG = GraspConfig(
        position_tolerance_m=0.015,
        orientation_tolerance_deg=12.0,
        contact_width_tolerance_mm=2.0,
    )
    return locals()
