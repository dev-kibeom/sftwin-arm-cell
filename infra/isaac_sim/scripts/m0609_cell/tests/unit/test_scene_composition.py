from types import SimpleNamespace
from pathlib import Path

import scene_builder.amr as amr
from scene_builder.amr import build_amr
from scene_builder.cell_metadata import initialize_cell_metadata
from scene_builder.composition import CELL_ROOT, LEGACY_CLEANUP
from scene_builder.rawpart_profile import load_rawpart_profile
from scene_builder.reachability import (
    planar_distance_xy,
    report_reach,
    spatial_distance_xyz,
)


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


def test_amr_scene_uses_operational_profile_for_raw_slot_fiducial(monkeypatch):
    created_paths = []
    authored_boxes = []
    authored_dynamic_boxes = []
    profile = load_rawpart_profile(Path(__file__).resolve().parents[6])

    def record_support_fiducial(context, passed_profile, support_origin):
        assert context.rawpart_profile is passed_profile
        created_paths.append(f"{CELL_ROOT}/AMR/Mockup/RawSlotSupportFiducial")
        authored_boxes.append(("support_origin", support_origin))

    monkeypatch.setattr(amr, "author_support_fiducial", record_support_fiducial)

    def record_box(path, *args, **kwargs):
        created_paths.append(path)
        authored_boxes.append((path, args))

    def record_dynamic_box(path, name, position, dimensions, *args, **kwargs):
        created_paths.append(path)
        authored_dynamic_boxes.append((path, name, position, dimensions, args, kwargs))

    context = SimpleNamespace(
        stage=object(),
        cell_root=CELL_ROOT,
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

    assert f"{CELL_ROOT}/AMR/Mockup/RawPart" in created_paths
    assert f"{CELL_ROOT}/AMR/Mockup/RawSlotSupportFiducial" in created_paths
    raw_slot = next(
        args
        for path, args in authored_boxes
        if path == f"{CELL_ROOT}/AMR/Mockup/TraySlots/RawSlot"
    )
    assert raw_slot[2] == [0.24, 0.28, 0.008]
    assert ("support_origin", (0.03, -0.85, 0.67)) in authored_boxes
    raw_part = next(
        entry
        for entry in authored_dynamic_boxes
        if entry[0] == f"{CELL_ROOT}/AMR/Mockup/RawPart"
    )
    _, _, position, dimensions, args, kwargs = raw_part
    assert dimensions == profile["dimensions_m"]
    assert position == [0.03, -0.85, 0.67 + dimensions[2] / 2.0]
    assert args[0] == context.colors["raw"]
    assert kwargs["material"] is context.materials["mat_raw"]
