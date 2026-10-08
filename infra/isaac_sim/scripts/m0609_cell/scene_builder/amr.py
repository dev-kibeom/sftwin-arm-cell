"""AMR mockup, tray, workpiece, docking target, and AMR-local metadata."""

from scene_builder.rawpart_profile import author_support_fiducial


def build_amr(context):
    stage = context.stage
    CELL_ROOT = context.cell_root
    COLORS = context.colors
    fixed_box = context.fixed_box
    static_box = context.static_box
    create_cylinder = context.create_cylinder
    set_custom_data = context.set_custom_data
    set_vec3_attribute = context.set_vec3_attribute
    dynamic_box = context.dynamic_box
    mat_frame = context.materials["mat_frame"]
    mat_steel_dark = context.materials["mat_steel_dark"]
    mat_vision = context.materials["mat_vision"]
    mat_raw = context.materials["mat_raw"]
    mat_finished = context.materials["mat_finished"]
    mat_white = context.materials["mat_white"]
    mat_black = context.materials["mat_black"]
    # -----------------------------------------------------------------------------
    # 5. AMR dock + direct robot pickup tray
    # -----------------------------------------------------------------------------
    amr_x, amr_y = 0.20, -0.85
    amr_tray_z = 0.64

    static_box(
        "amr_base",
        f"{CELL_ROOT}/AMR/Mockup/Base",
        "amr_base",
        COLORS["charcoal"],
        mat_frame,
    )

    static_box(
        "amr_upper_body",
        f"{CELL_ROOT}/AMR/Mockup/UpperBody",
        "amr_upper_body",
        COLORS["steel_dark"],
        mat_steel_dark,
    )

    static_box(
        "amr_tray_lift",
        f"{CELL_ROOT}/AMR/Mockup/TrayLift",
        "amr_tray_lift",
        COLORS["frame"],
        mat_frame,
    )

    static_box(
        "amr_tray",
        f"{CELL_ROOT}/AMR/Mockup/Tray",
        "amr_tray",
        COLORS["vision"],
        mat_vision,
    )

    for object_id, name in [
        ("amr_tray_rail_left", "LeftRail"),
        ("amr_tray_rail_right", "RightRail"),
        ("amr_tray_rail_front", "FrontRail"),
        ("amr_tray_rail_rear", "RearRail"),
    ]:
        static_box(
            object_id,
            f"{CELL_ROOT}/AMR/Mockup/TrayRails/{name}",
            name,
            COLORS["frame"],
            mat_frame,
        )

    raw_slot_x = amr_x - 0.17
    done_slot_x = amr_x + 0.17
    slot_y = amr_y

    static_box(
        "amr_raw_slot_support",
        f"{CELL_ROOT}/AMR/Mockup/TraySlots/RawSlot",
        "raw_slot",
        COLORS["raw"],
        mat_raw,
    )
    author_support_fiducial(
        context,
        context.rawpart_profile,
        (raw_slot_x, slot_y, amr_tray_z + 0.030),
    )

    rawpart_dimensions = context.rawpart_profile["dimensions_m"]
    raw_slot_top_z = amr_tray_z + 0.030
    dynamic_box(
        f"{CELL_ROOT}/AMR/Mockup/RawPart",
        "raw_part",
        [
            raw_slot_x,
            slot_y,
            raw_slot_top_z + float(rawpart_dimensions[2]) / 2.0,
        ],
        rawpart_dimensions,
        COLORS["raw"],
        material=mat_raw,
    )

    static_box(
        "amr_finished_slot_support",
        f"{CELL_ROOT}/AMR/Mockup/TraySlots/FinishedSlot",
        "finished_slot",
        COLORS["finished"],
        mat_finished,
    )

    for i, (dx, dy) in enumerate(
        [
            (-0.29, -0.21),
            (-0.29, 0.21),
            (0.29, -0.21),
            (0.29, 0.21),
        ]
    ):
        create_cylinder(
            stage,
            f"{CELL_ROOT}/AMR/Mockup/Wheel_{i + 1}",
            0.075,
            0.045,
            [amr_x + dx, amr_y + dy, 0.10],
            COLORS["black"],
            mat_black,
            axis="X",
        )

    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/DockTarget",
        "dock_target",
        [amr_x, amr_y + 0.315, 0.37],
        [0.18, 0.018, 0.12],
        COLORS["white"],
        mat_white,
    )
    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/DockTargetCenter",
        "dock_target_center",
        [amr_x, amr_y + 0.325, 0.37],
        [0.055, 0.008, 0.055],
        COLORS["black"],
        mat_black,
    )

    set_custom_data(stage, f"{CELL_ROOT}/AMR/Mockup", "sf_twin:role", "amr_placeholder")
    set_vec3_attribute(
        stage, f"{CELL_ROOT}/AMR/Mockup", "sf_twin:dock_pose", [amr_x, amr_y, 0.0]
    )
    set_custom_data(
        stage, f"{CELL_ROOT}/AMR/Mockup/Tray", "sf_twin:role", "direct_pick_tray"
    )
    set_custom_data(
        stage,
        f"{CELL_ROOT}/AMR/Mockup/TraySlots/FinishedSlot",
        "sf_twin:role",
        "finished_part_return_slot",
    )

    context.amr_x = amr_x
    context.amr_y = amr_y
    context.amr_tray_z = amr_tray_z
    context.raw_slot_x = raw_slot_x
    context.done_slot_x = done_slot_x
    context.slot_y = slot_y
