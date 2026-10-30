from types import SimpleNamespace
from pathlib import Path
import json

import scene_builder.amr as amr
from scene_builder.amr import build_amr
from scene_builder.cell_metadata import initialize_cell_metadata
from scene_builder.composition import CELL_ROOT, LEGACY_CLEANUP
from scene_builder.rawpart_profile import load_rawpart_profile
from scene_builder.static_environment import load_canonical_static_geometry
from scene_builder.reachability import (
    planar_distance_xy,
    report_reach,
    spatial_distance_xyz,
)


def _canonical_tray_support_origin(project_root):
    manifest = json.loads(
        (project_root / "config/m0609_static_environment.json").read_text()
    )
    objects = {item["id"]: item for item in manifest["objects"]}
    tray = objects["amr_tray"]
    center = tray["pose"]["position_m"]
    dimensions = tray["geometry"]["dimensions_m"]
    return (center[0], center[1], center[2] + dimensions[2] / 2)


def test_cell_metadata_initialization_owns_root_children_and_identity_metadata():
    ensured = []
    metadata = []
    context = SimpleNamespace(
        stage=object(),
        cell_root=CELL_ROOT,
        ensure_xform=lambda stage, path: ensured.append((stage, path)),
        set_custom_data=lambda stage, path, key, value: metadata.append(
            (stage, path, key, value)
        ),
    )

    initialize_cell_metadata(context)

    assert [path for _, path in ensured] == [
        CELL_ROOT,
        *[
            f"{CELL_ROOT}/{child}"
            for child in (
                "Environment",
                "Safety",
                "RobotStation",
                "Vision",
                "CameraRig",
                "CNC",
                "AMR",
                "Status",
                "Materials",
            )
        ],
    ]
    assert metadata == [
        (context.stage, CELL_ROOT, "sf_twin:cell_id", "CELL-A01"),
        (
            context.stage,
            CELL_ROOT,
            "sf_twin:process",
            "direct_amr_to_cnc_machine_tending",
        ),
        (context.stage, CELL_ROOT, "sf_twin:version", "v4.2"),
    ]


def test_reachability_policy_preserves_distances_and_report_thresholds():
    assert planar_distance_xy([0, 0, 9], [3, 4, -2]) == 5.0
    assert spatial_distance_xyz([0, 0, 0], [2, 3, 6]) == 7.0
    reports = []
    report_reach("comfortable", 0.75, 0.8, reports.append)
    report_reach("boundary", 0.76, 0.8, reports.append)
    report_reach("outside", 0.91, 0.8, reports.append)
    assert reports[0].startswith(">>> [KINEMATIC OK]")
    assert reports[1].startswith(">>> [KINEMATIC WARN]")
    assert reports[2].startswith(">>> [KINEMATIC FAIL]")


def test_composition_retains_legacy_cleanup_order_and_canonical_cell_root():
    assert CELL_ROOT == "/World/SF_Twin_Cell"
    assert LEGACY_CLEANUP == [
        "/World/SF_Twin_Cell",
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


def test_amr_scene_uses_tray_and_front_rail_marker(monkeypatch):
    project_root = Path(__file__).resolve().parents[6]
    support_origin = _canonical_tray_support_origin(project_root)
    created_paths = []
    authored_boxes = []
    authored_dynamic_boxes = []
    profile = load_rawpart_profile(project_root)

    def record_support_fiducial(context, passed_profile, support_origin):
        assert context.rawpart_profile is passed_profile
        created_paths.append(f"{CELL_ROOT}/AMR/Mockup/TraySupportFiducial")
        authored_boxes.append(("support_origin", support_origin))

    monkeypatch.setattr(amr, "author_support_fiducial", record_support_fiducial)

    def record_box(path, *args, **kwargs):
        created_paths.append(path)
        authored_boxes.append((path, args))

    def record_dynamic_box(path, name, position, dimensions, *args, **kwargs):
        created_paths.append(path)
        authored_dynamic_boxes.append((path, name, position, dimensions, args, kwargs))

    def record_static_box(object_id, path, name, color, material):
        record_box(path, object_id, name, color, material)

    context = SimpleNamespace(
        stage=object(),
        cell_root=CELL_ROOT,
        static_geometry=load_canonical_static_geometry(project_root=project_root),
        rawpart_profile=profile,
        colors={
            name: [0.0, 0.0, 0.0]
            for name in (
                "charcoal",
                "steel_dark",
                "frame",
                "vision",
                "raw",
                "finished",
                "kraft",
                "white",
                "black",
            )
        },
        fixed_box=record_box,
        static_box=record_static_box,
        dynamic_box=record_dynamic_box,
        create_cylinder=lambda *args, **kwargs: None,
        set_custom_data=lambda *args, **kwargs: None,
        set_vec3_attribute=lambda *args, **kwargs: None,
        materials={
            name: object()
            for name in (
                "mat_frame",
                "mat_steel_dark",
                "mat_vision",
                "mat_raw",
                "mat_finished",
                "mat_kraft",
                "mat_white",
                "mat_black",
            )
        },
    )

    build_amr(context)

    assert f"{CELL_ROOT}/AMR/Mockup/RawPart" not in created_paths
    assert f"{CELL_ROOT}/AMR/Mockup/TraySupportFiducial" in created_paths
    assert f"{CELL_ROOT}/AMR/Mockup/TrayMarkerPost" in created_paths
    assert not any("TraySlots" in path for path in created_paths)
    # Marker post is a static manifest object.
    assert any(
        item[0] == f"{CELL_ROOT}/AMR/Mockup/TrayMarkerPost" for item in authored_boxes
    )
    assert ("support_origin", support_origin) in authored_boxes


def test_tray_marker_post_joins_front_rail_to_marker_underside():
    project_root = Path(__file__).resolve().parents[6]
    manifest = json.loads(
        (project_root / "config/m0609_static_environment.json").read_text()
    )
    objects = {item["id"]: item for item in manifest["objects"]}
    front_rail = objects["amr_tray_rail_front"]
    marker_post = objects["amr_tray_marker_post"]
    profile = load_rawpart_profile(project_root)

    post_center = marker_post["pose"]["position_m"]
    half_post_height = marker_post["geometry"]["dimensions_m"][2] / 2.0
    rail_top = (
        front_rail["pose"]["position_m"][2]
        + front_rail["geometry"]["dimensions_m"][2] / 2.0
    )
    marker_relation = profile["fiducial"]["marker_in_support"]["position_m"]
    support_origin = _canonical_tray_support_origin(project_root)
    marker_center_xy = (
        support_origin[0] + marker_relation[0],
        support_origin[1] + marker_relation[1],
    )
    marker_plane_z = support_origin[2] + marker_relation[2]
    marker_half_size = profile["fiducial"]["size_m"] / 2.0

    assert abs(post_center[2] - half_post_height - rail_top) < 1e-9
    assert abs(marker_plane_z - (post_center[2] + half_post_height) - 0.0001) < 1e-9
    assert abs(post_center[0] - marker_center_xy[0]) <= marker_half_size
    assert abs(post_center[1] - marker_center_xy[1]) <= marker_half_size
