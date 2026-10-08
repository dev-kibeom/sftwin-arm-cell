"""D455 asset reference, mount, and rigid-body handling."""

D455_COLOR_CAMERA_NAME = "Camera_OmniVision_OV9782_Color"
D455_ASSET_RELATIVE_PATH = "Isaac/Sensors/Intel/RealSense/rsd455.usd"
D455_ASSET_FALLBACK_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
EMBEDDED_IMU_PRIM_NAME = "Imu_Sensor"


def d455_asset_url(assets_root):
    if assets_root:
        return f"{assets_root}/{D455_ASSET_RELATIVE_PATH}"
    return D455_ASSET_FALLBACK_URL


def _pxr_modules():
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    return Gf, Usd, UsdGeom, UsdPhysics


def disable_rigid_bodies(root_prim, *, usd=None, usd_physics=None):
    """Disable every RigidBodyAPI within a referenced D455 subtree."""
    if usd is None or usd_physics is None:
        _, default_usd, _, default_usd_physics = _pxr_modules()
        usd = usd or default_usd
        usd_physics = usd_physics or default_usd_physics

    disabled = []
    for prim in usd.PrimRange(root_prim):
        if not prim.IsValid():
            continue
        if prim.HasAPI(usd_physics.RigidBodyAPI):
            rb_api = usd_physics.RigidBodyAPI(prim)
            attr = rb_api.GetRigidBodyEnabledAttr()
            if not attr:
                attr = rb_api.CreateRigidBodyEnabledAttr()
            attr.Set(False)
            disabled.append(str(prim.GetPath()))
    return disabled


def disable_embedded_imu(root_prim, *, usd=None):
    """Deactivate unused IMU prims embedded in a visual-only camera asset."""
    if usd is None:
        _, usd, _, _ = _pxr_modules()

    disabled = []
    for prim in list(usd.PrimRange(root_prim)):
        if not prim.IsValid() or prim.GetName() != EMBEDDED_IMU_PRIM_NAME:
            continue
        prim.SetActive(False)
        disabled.append(str(prim.GetPath()))
    return disabled


def mount_d455(
    stage,
    asset_url,
    prim_path,
    position,
    rotation_xyz_deg,
    *,
    add_reference,
    gf=None,
    usd_geom=None,
    embedded_imu_disabler=None,
    rigid_body_disabler=disable_rigid_bodies,
    report=print,
):
    """Reference and mount the D455 while preserving the original authoring order."""
    if gf is None or usd_geom is None:
        default_gf, _, default_usd_geom, _ = _pxr_modules()
        gf = gf or default_gf
        usd_geom = usd_geom or default_usd_geom

    add_reference(usd_path=asset_url, prim_path=prim_path, prim_type="Xform")
    d455_prim = stage.GetPrimAtPath(prim_path)
    if not d455_prim.IsValid():
        report(">>> [WARN] RealSense D455 prim was not created")
        return d455_prim, []

    d455_xform = usd_geom.Xformable(d455_prim)
    d455_xform.ClearXformOpOrder()
    d455_xform.AddTranslateOp(precision=usd_geom.XformOp.PrecisionDouble).Set(
        gf.Vec3d(float(position[0]), float(position[1]), float(position[2]))
    )
    d455_xform.AddRotateXYZOp().Set(
        gf.Vec3f(
            float(rotation_xyz_deg[0]),
            float(rotation_xyz_deg[1]),
            float(rotation_xyz_deg[2]),
        )
    )
    if embedded_imu_disabler is None:
        disabled_imu_paths = disable_embedded_imu(d455_prim)
    else:
        disabled_imu_paths = embedded_imu_disabler(d455_prim)
    disabled_paths = rigid_body_disabler(d455_prim)
    d455_prim.SetCustomDataByKey("sf_twin:role", "realsense_d455_visual")
    report(">>> [SUCCESS] RealSense D455 mounted below compact bracket")
    if disabled_imu_paths:
        report(">>> [INFO] Unused D455 IMU sensors disabled:")
        for path in disabled_imu_paths:
            report(f"    - {path}")
    if disabled_paths:
        report(">>> [INFO] D455 rigid bodies disabled:")
        for path in disabled_paths:
            report(f"    - {path}")
    else:
        report(">>> [INFO] No RigidBodyAPI found inside D455 reference")
    return d455_prim, disabled_paths
