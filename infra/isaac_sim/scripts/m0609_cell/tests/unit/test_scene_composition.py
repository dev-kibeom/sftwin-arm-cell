from types import SimpleNamespace

from scene_builder.cell_metadata import initialize_cell_metadata
from scene_builder.composition import CELL_ROOT, LEGACY_CLEANUP
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
