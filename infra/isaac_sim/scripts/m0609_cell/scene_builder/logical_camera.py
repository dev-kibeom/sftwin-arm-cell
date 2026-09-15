"""Logical Camera_Sensor construction owned by the M0609 scene builder."""

from scene_builder.camera_extrinsic import author_camera_sensor_from_d455_color


CAMERA_SENSOR_RELATIVE_PATH = "/Vision/Camera_Sensor"
CAMERA_SENSOR_FOCAL_LENGTH = 24.0
CAMERA_SENSOR_CLIPPING_RANGE_M = (0.10, 10.0)
CAMERA_SENSOR_ROLE = "amr_pickup_overhead_camera"
CAMERA_SENSOR_OBSERVES = "amr_tray_and_workpiece"


def _pxr_modules():
    from pxr import Gf, UsdGeom

    return Gf, UsdGeom


def create_logical_camera_sensor(
    stage,
    cell_root,
    d455_root_path,
    *,
    gf=None,
    usd_geom=None,
    extrinsic_author=author_camera_sensor_from_d455_color,
):
    """Create and configure the logical ROS camera, then author its D455 extrinsic."""
    if gf is None or usd_geom is None:
        default_gf, default_usd_geom = _pxr_modules()
        gf = gf or default_gf
        usd_geom = usd_geom or default_usd_geom
    camera_path = f"{cell_root}{CAMERA_SENSOR_RELATIVE_PATH}"
    camera = usd_geom.Camera.Define(stage, camera_path)
    extrinsic_author(stage, camera.GetPrim(), d455_root_path)
    camera.GetFocalLengthAttr().Set(CAMERA_SENSOR_FOCAL_LENGTH)
    camera.GetClippingRangeAttr().Set(gf.Vec2f(*CAMERA_SENSOR_CLIPPING_RANGE_M))
    camera.GetPrim().SetCustomDataByKey("sf_twin:role", CAMERA_SENSOR_ROLE)
    camera.GetPrim().SetCustomDataByKey("sf_twin:observes", CAMERA_SENSOR_OBSERVES)
    return camera
