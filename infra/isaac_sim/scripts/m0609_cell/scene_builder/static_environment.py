"""Pure canonical static-environment manifest contract."""

import json
import os
from pathlib import Path


STATIC_MANIFEST_RELATIVE_PATH = Path("config/m0609_static_environment.json")
REQUIRED_STATIC_OBJECT_IDS = {
    "factory_floor",
    "pedestal",
    "pedestal_top",
    "robot_riser",
    "cnc_base",
    "cnc_upper_housing",
    "cnc_front_left",
    "cnc_front_top",
    "camera_rig_post_left",
    "camera_rig_post_right",
    "camera_rig_top_beam",
    "camera_boom",
    "camera_drop_bracket",
    "camera_bracket_arm",
    "cable_tray",
    "amr_base",
    "amr_upper_body",
    "amr_tray_lift",
    "amr_tray",
    "amr_tray_rail_left",
    "amr_tray_rail_right",
    "amr_tray_rail_front",
    "amr_tray_rail_rear",
    "amr_tray_marker_post",
}


def canonical_manifest_path(project_root=None, script_path=None):
    """Resolve the manifest without assuming a Script Editor ``__file__`` depth."""
    configured_root = project_root or os.environ.get("SFTWIN_PROJECT_ROOT")
    if configured_root:
        root = Path(configured_root).expanduser()
        if not root.is_dir():
            raise RuntimeError(f"SFTWIN_PROJECT_ROOT is not a directory: {root}")
        manifest_path = root / STATIC_MANIFEST_RELATIVE_PATH
        if not manifest_path.is_file():
            raise RuntimeError(
                f"SFTWIN_PROJECT_ROOT does not contain {STATIC_MANIFEST_RELATIVE_PATH}: {root}"
            )
        return manifest_path

    source_path = Path(script_path if script_path is not None else __file__).resolve()
    for candidate_root in (source_path.parent, *source_path.parents):
        manifest_path = candidate_root / STATIC_MANIFEST_RELATIVE_PATH
        if manifest_path.is_file():
            return manifest_path
    raise RuntimeError(
        "Cannot locate canonical static manifest from this script path. "
        "When running from Isaac Script Editor, set SFTWIN_PROJECT_ROOT to the repository root."
    )


def load_canonical_static_geometry(project_root=None, script_path=None, *, manifest_path=None):
    manifest_path = (Path(manifest_path) if manifest_path is not None
                     else canonical_manifest_path(project_root, script_path))
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema_version") != 1 or manifest.get("coverage") != "partial":
        raise RuntimeError(f"Unsupported static environment manifest: {manifest_path}")
    objects = {obj["id"]: obj for obj in manifest["objects"]}
    if set(objects) != REQUIRED_STATIC_OBJECT_IDS:
        raise RuntimeError("Canonical static environment does not match builder scope")
    return objects


def author_static_box(
    static_geometry, fixed_box, object_id, path, name, color, material=None
):
    """Author one manifest-backed static box through the supplied primitive writer."""
    obj = static_geometry[object_id]
    return fixed_box(
        path,
        name,
        obj["pose"]["position_m"],
        obj["geometry"]["dimensions_m"],
        color,
        material,
    )


def supply_support_origin(static_geometry, binding):
    """Supply falls onto the selected existing support's upper box surface."""
    support = static_geometry[binding['support_geometry_object']]
    x, y, z = support['pose']['position_m']
    return (x, y, z + support['geometry']['dimensions_m'][2] / 2.0)


def camera_mount_pose(static_geometry, camera):
    """Resolve the Cell mount offset against its existing geometry origin."""
    mount = camera['mount']
    origin = static_geometry[mount['geometry_object']]['pose']['position_m']
    return ([a + b for a, b in zip(origin, mount['translation_offset_m'])],
            mount['rotation_xyz_deg'])
