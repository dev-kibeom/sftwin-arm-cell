"""Logical Camera_Sensor construction owned by the M0609 scene builder."""

from scene_builder.camera_extrinsic import author_camera_sensor_from_d455_color


CAMERA_SENSOR_RELATIVE_PATH = "/Vision/Camera_Sensor"
CAMERA_SENSOR_FOCAL_LENGTH = 24.0
CAMERA_SENSOR_CLIPPING_RANGE_M = (0.10, 10.0)
CAMERA_SENSOR_RESOLUTION = (1280, 720)
CAMERA_SENSOR_ROLE = "amr_pickup_overhead_camera"
CAMERA_SENSOR_OBSERVES = "amr_tray_and_workpiece"
OPENCV_PINHOLE_API = "OmniLensDistortionOpenCvPinholeAPI"
OPENCV_PINHOLE_PREFIX = "omni:lensdistortion:opencvPinhole:"
OPENCV_PINHOLE_ZERO_COEFFICIENTS = (
    "k1",
    "k2",
    "p1",
    "p2",
    "k3",
    "k4",
    "k5",
    "k6",
    "s1",
    "s2",
    "s3",
    "s4",
)


def _pxr_modules():
    from pxr import Gf, UsdGeom

    return Gf, UsdGeom


def _author_square_pixel_pinhole(camera_prim, focal_length, horizontal_aperture, gf):
    width, height = CAMERA_SENSOR_RESOLUTION
    vertical_aperture = horizontal_aperture * height / width
    camera_prim.GetAttribute("verticalAperture").Set(vertical_aperture)

    if not camera_prim.HasAPI(OPENCV_PINHOLE_API):
        camera_prim.ApplyAPI(OPENCV_PINHOLE_API)
    if not camera_prim.HasAPI(OPENCV_PINHOLE_API):
        raise RuntimeError(
            f"failed to apply camera distortion schema {OPENCV_PINHOLE_API}"
        )

    focal_px = focal_length * width / horizontal_aperture
    values = {
        "omni:lensdistortion:model": "opencvPinhole",
        f"{OPENCV_PINHOLE_PREFIX}cx": width / 2.0,
        f"{OPENCV_PINHOLE_PREFIX}cy": height / 2.0,
        f"{OPENCV_PINHOLE_PREFIX}fx": focal_px,
        f"{OPENCV_PINHOLE_PREFIX}fy": focal_px,
        f"{OPENCV_PINHOLE_PREFIX}imageSize": gf.Vec2i(width, height),
    }
    values.update(
        {
            f"{OPENCV_PINHOLE_PREFIX}{name}": 0.0
            for name in OPENCV_PINHOLE_ZERO_COEFFICIENTS
        }
    )
    for name, value in values.items():
        camera_prim.GetAttribute(name).Set(value)


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
    _author_square_pixel_pinhole(
        camera.GetPrim(),
        CAMERA_SENSOR_FOCAL_LENGTH,
        camera.GetHorizontalApertureAttr().Get(),
        gf,
    )
    camera.GetPrim().SetCustomDataByKey("sf_twin:role", CAMERA_SENSOR_ROLE)
    camera.GetPrim().SetCustomDataByKey("sf_twin:observes", CAMERA_SENSOR_OBSERVES)
    return camera
