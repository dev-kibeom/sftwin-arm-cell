"""Final PnP_CNC scene composition, cleanup, and ordered builder orchestration."""

from dataclasses import dataclass
import os
from pathlib import Path

import numpy as np

from .amr import build_amr
from .cell_metadata import (
    apply_integration_metadata,
    build_status_hardware,
    initialize_cell_metadata,
)
from .d455_camera import local_d455_asset, mount_d455
from .environment import build_environment
from .logical_camera import create_logical_camera_sensor
from .reachability import run_reachability_check
from .rawpart_profile import validate_rawpart_profile
from .robot_station import build_robot_station
from .materials import create_preview_material
from .primitives import (
    create_cylinder,
    create_sphere,
    dynamic_box,
    fixed_box,
)
from .static_environment import (
    author_static_box,
    camera_mount_pose,
    load_canonical_static_geometry,
)
from .usd_authoring import (
    ensure_xform,
    set_custom_data,
    set_vec3_attribute,
)


CELL_ROOT = "/World/SF_Twin_Cell"
COLORS = {
    "floor": np.array([0.075, 0.082, 0.090]),
    "cell_floor": np.array([0.12, 0.13, 0.145]),
    "charcoal": np.array([0.055, 0.060, 0.070]),
    "frame": np.array([0.09, 0.10, 0.115]),
    "steel": np.array([0.28, 0.30, 0.33]),
    "steel_dark": np.array([0.16, 0.18, 0.20]),
    "safety": np.array([0.78, 0.56, 0.08]),
    "vision": np.array([0.025, 0.065, 0.13]),
    "kraft": np.array([0.52, 0.36, 0.20]),
    "red": np.array([0.65, 0.03, 0.03]),
    "amber": np.array([0.85, 0.35, 0.03]),
    "green": np.array([0.03, 0.55, 0.10]),
    "blue": np.array([0.05, 0.22, 0.55]),
    "cyan": np.array([0.03, 0.45, 0.55]),
    "white": np.array([0.85, 0.87, 0.90]),
    "black": np.array([0.025, 0.025, 0.03]),
    "acrylic": np.array([0.16, 0.18, 0.22]),
}

LEGACY_CLEANUP = [
    CELL_ROOT,
    "/World/Industrial_Floor",
    "/World/Floor_Tiles",
    "/World/Safety_Zone_Marking",
    "/World/Safety_Fence",
    "/World/StationA_Table",
    "/World/CameraRig",
    "/World/Pallet_Tray",
    "/World/box_item_01",
    "/World/Camera_Sensor",
    "/World/m0609",
]


@dataclass
class SceneBuildContext:
    stage: object
    gf: object
    usd_geom: object
    usd_lux: object
    colors: object
    cell_root: str
    ensure_xform: object
    set_custom_data: object
    set_vec3_attribute: object
    create_preview_material: object
    fixed_box: object
    dynamic_box: object
    create_cylinder: object
    create_sphere: object
    static_box: object
    add_reference_to_stage: object
    static_geometry: dict


def build_camera_rig(context):
    stage, cell_root, colors = context.stage, context.cell_root, context.colors
    mat_frame, mat_black = (
        context.materials["mat_frame"],
        context.materials["mat_black"],
    )
    static_box = context.static_box
    for object_id, path, name, color, material in [
        (
            "camera_rig_post_left",
            f"{cell_root}/CameraRig/PostLeft",
            "PostLeft",
            colors["frame"],
            mat_frame,
        ),
        (
            "camera_rig_post_right",
            f"{cell_root}/CameraRig/PostRight",
            "PostRight",
            colors["frame"],
            mat_frame,
        ),
        (
            "camera_rig_top_beam",
            f"{cell_root}/CameraRig/TopBeam",
            "camera_top_beam",
            colors["frame"],
            mat_frame,
        ),
        (
            "camera_boom",
            f"{cell_root}/CameraRig/CameraBoom",
            "camera_boom",
            colors["frame"],
            mat_frame,
        ),
        (
            "camera_drop_bracket",
            f"{cell_root}/CameraRig/CameraDropBracket",
            "camera_drop_bracket",
            colors["black"],
            mat_black,
        ),
        (
            "camera_bracket_arm",
            f"{cell_root}/CameraRig/CameraBracketArm",
            "camera_bracket_arm",
            colors["black"],
            mat_black,
        ),
        (
            "cable_tray",
            f"{cell_root}/CameraRig/CableTray",
            "camera_cable_tray",
            colors["charcoal"],
            mat_black,
        ),
    ]:
        static_box(object_id, path, name, color, material)
    realsense_url = local_d455_asset()
    camera = context.camera_configuration
    rs_path = camera['mount_prim_path']
    mount_position, mount_rotation = camera_mount_pose(context.static_geometry, camera)
    mount_d455(
        stage,
        realsense_url,
        rs_path,
        mount_position,
        mount_rotation,
        add_reference=context.add_reference_to_stage,
    )
    create_logical_camera_sensor(stage, cell_root, rs_path, camera_configuration=camera)


def run_scene_build(
    *,
    world_cls,
    enable_extension,
    delete_prim,
    add_reference_to_stage,
    gf,
    usd_geom,
    usd_lux,
    script_path,
    configuration,
):
    """Build the complete scene in the original cleanup and authoring order."""
    enable_extension("isaacsim.asset.importer.urdf")
    enable_extension("isaacsim.ros2.bridge")
    world = world_cls.instance()
    if world is None:
        world = world_cls(stage_units_in_meters=1.0)
    stage = world.stage
    from .scene_configuration import SceneConfiguration

    project_root = Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()
    selected = SceneConfiguration.from_runtime(configuration, project_root)
    static_geometry = load_canonical_static_geometry(manifest_path=selected.geometry)

    def author_fixed_box(path, name, position, scale, color, material=None):
        return fixed_box(stage, path, name, position, scale, color, material)

    def author_dynamic_box(path, name, position, scale, color, mass=0.4, material=None):
        return dynamic_box(stage, path, name, position, scale, color, mass, material)

    def author_static_manifest_box(object_id, path, name, color, material=None):
        return author_static_box(
            static_geometry, author_fixed_box, object_id, path, name, color, material
        )

    context = SceneBuildContext(
        stage,
        gf,
        usd_geom,
        usd_lux,
        COLORS,
        CELL_ROOT,
        ensure_xform,
        set_custom_data,
        set_vec3_attribute,
        lambda stage, name, color, roughness=0.45, metallic=0.0, opacity=1.0: create_preview_material(
            stage, CELL_ROOT, name, color, roughness, metallic, opacity
        ),
        author_fixed_box,
        author_dynamic_box,
        create_cylinder,
        create_sphere,
        author_static_manifest_box,
        add_reference_to_stage,
        static_geometry,
    )
    context.camera_configuration = configuration['cell']['camera']
    context.scene_configuration = selected
    context.rawpart_profile = validate_rawpart_profile(configuration)
    for path in LEGACY_CLEANUP:
        if context.stage.GetPrimAtPath(path).IsValid():
            delete_prim(path)
    initialize_cell_metadata(context)
    build_environment(context)
    build_robot_station(context)
    build_amr(context)
    build_camera_rig(context)
    build_status_hardware(context)
    apply_integration_metadata(context)
    run_reachability_check(context)
    print(
        "ARM Cell simulation scene is ready for material transfer and robot operation."
    )
    return context
