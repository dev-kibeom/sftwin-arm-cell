"""D455 Color composed-pose lookup and logical-camera extrinsic authoring."""

import json

from scene_builder.camera_alignment_policy import near_rigid_rotation
from scene_builder.d455_camera import D455_COLOR_CAMERA_NAME


def _pxr_modules():
    from pxr import Gf, Usd, UsdGeom

    return Gf, Usd, UsdGeom


def find_d455_color_camera(stage, d455_root_path, *, usd=None, usd_geom=None):
    """Find the one expected D455 Color camera below the referenced assembly."""
    if usd is None or usd_geom is None:
        _, default_usd, default_usd_geom = _pxr_modules()
        usd = usd or default_usd
        usd_geom = usd_geom or default_usd_geom
    root = stage.GetPrimAtPath(d455_root_path)
    if not root.IsValid():
        raise RuntimeError(f"D455 root is missing: {d455_root_path}")
    matches = [
        prim
        for prim in usd.PrimRange(root)
        if prim.IsA(usd_geom.Camera) and prim.GetName() == D455_COLOR_CAMERA_NAME
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one D455 Color camera named {D455_COLOR_CAMERA_NAME}; found {len(matches)}"
        )
    return matches[0]


def d455_color_rigid_world_transform(
    stage, d455_root_path, *, gf=None, usd=None, usd_geom=None
):
    """Derive the accepted proper rigid D455 Color transform in world coordinates."""
    if gf is None or usd is None or usd_geom is None:
        default_gf, default_usd, default_usd_geom = _pxr_modules()
        gf = gf or default_gf
        usd = usd or default_usd
        usd_geom = usd_geom or default_usd_geom
    color_camera = find_d455_color_camera(
        stage, d455_root_path, usd=usd, usd_geom=usd_geom
    )
    composed = usd_geom.XformCache().GetLocalToWorldTransform(color_camera)
    raw_rotation = [
        [float(composed[row][column]) for column in range(3)] for row in range(3)
    ]
    rotation, evidence = near_rigid_rotation(raw_rotation)
    translation = [float(composed[3][index]) for index in range(3)]
    rigid = gf.Matrix4d(1.0)
    for row in range(3):
        rigid.SetRow(
            row,
            gf.Vec4d(
                float(rotation[row][0]),
                float(rotation[row][1]),
                float(rotation[row][2]),
                0.0,
            ),
        )
    rigid.SetRow(3, gf.Vec4d(*translation, 1.0))
    return rigid, str(color_camera.GetPath()), evidence


def author_camera_sensor_from_d455_color(
    stage,
    camera_prim,
    d455_root_path,
    *,
    gf=None,
    usd=None,
    usd_geom=None,
    report=print,
):
    """Author Camera_Sensor from the accepted D455 Color world transform."""
    if gf is None or usd is None or usd_geom is None:
        default_gf, default_usd, default_usd_geom = _pxr_modules()
        gf = gf or default_gf
        usd = usd or default_usd
        usd_geom = usd_geom or default_usd_geom
    world_transform, color_path, evidence = d455_color_rigid_world_transform(
        stage, d455_root_path, gf=gf, usd=usd, usd_geom=usd_geom
    )
    parent = camera_prim.GetParent()
    parent_world = usd_geom.XformCache().GetLocalToWorldTransform(parent)
    local_transform = world_transform * parent_world.GetInverse()
    camera_xform = usd_geom.Xformable(camera_prim)
    camera_xform.ClearXformOpOrder()
    camera_xform.AddTransformOp(precision=usd_geom.XformOp.PrecisionDouble).Set(
        local_transform
    )
    report(
        f">>> [SUCCESS] Camera_Sensor pose derived from accepted D455 Color pose: {color_path}"
    )
    report(
        f">>> [INFO] D455 near-rigid evidence: {json.dumps(evidence, sort_keys=True)}"
    )
    return color_path, evidence
