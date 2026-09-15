"""AMR mockup, tray, workpiece, docking target, and AMR-local metadata."""


def build_amr(context):
    stage = context.stage
    CELL_ROOT = context.cell_root
    COLORS = context.colors
    fixed_box = context.fixed_box
    dynamic_box = context.dynamic_box
    create_cylinder = context.create_cylinder
    set_custom_data = context.set_custom_data
    set_vec3_attribute = context.set_vec3_attribute
    mat_frame = context.materials["mat_frame"]
    mat_steel_dark = context.materials["mat_steel_dark"]
    mat_vision = context.materials["mat_vision"]
    mat_raw = context.materials["mat_raw"]
    mat_finished = context.materials["mat_finished"]
    mat_kraft = context.materials["mat_kraft"]
    mat_white = context.materials["mat_white"]
    mat_black = context.materials["mat_black"]
    # -----------------------------------------------------------------------------
    # 5. AMR dock + direct robot pickup tray
    # -----------------------------------------------------------------------------
    amr_x, amr_y = 0.20, -0.85
    amr_tray_z = 0.64

    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/Base",
        "amr_base",
        [amr_x, amr_y, 0.17],
        [0.78, 0.60, 0.26],
        COLORS["charcoal"],
        mat_frame,
    )

    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/UpperBody",
        "amr_upper_body",
        [amr_x, amr_y, 0.36],
        [0.72, 0.56, 0.20],
        COLORS["steel_dark"],
        mat_steel_dark,
    )

    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/TrayLift",
        "amr_tray_lift",
        [amr_x, amr_y, 0.50],
        [0.56, 0.42, 0.16],
        COLORS["frame"],
        mat_frame,
    )

    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/Tray",
        "amr_tray",
        [amr_x, amr_y, amr_tray_z],
        [0.66, 0.46, 0.045],
        COLORS["vision"],
        mat_vision,
    )

    for name, pos, scale in [
        ("LeftRail", [amr_x - 0.325, amr_y, amr_tray_z + 0.035], [0.025, 0.46, 0.07]),
        ("RightRail", [amr_x + 0.325, amr_y, amr_tray_z + 0.035], [0.025, 0.46, 0.07]),
        ("FrontRail", [amr_x, amr_y - 0.225, amr_tray_z + 0.035], [0.66, 0.025, 0.07]),
        ("RearRail", [amr_x, amr_y + 0.225, amr_tray_z + 0.035], [0.66, 0.025, 0.07]),
    ]:
        fixed_box(
            f"{CELL_ROOT}/AMR/Mockup/TrayRails/{name}",
            name,
            pos,
            scale,
            COLORS["frame"],
            mat_frame,
        )

    raw_slot_x = amr_x - 0.17
    done_slot_x = amr_x + 0.17
    slot_y = amr_y

    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/TraySlots/RawSlot",
        "raw_slot",
        [raw_slot_x, slot_y, amr_tray_z + 0.026],
        [0.24, 0.28, 0.008],
        COLORS["raw"],
        mat_raw,
    )

    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/TraySlots/FinishedSlot",
        "finished_slot",
        [done_slot_x, slot_y, amr_tray_z + 0.026],
        [0.24, 0.28, 0.008],
        COLORS["finished"],
        mat_finished,
    )

    dynamic_box(
        f"{CELL_ROOT}/AMR/Mockup/RawPart",
        "raw_part",
        [raw_slot_x, slot_y, amr_tray_z + 0.085],
        [0.14, 0.14, 0.08],
        COLORS["kraft"],
        mass=0.45,
        material=mat_kraft,
    )

    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/RawPartMarker/Base",
        "raw_part_marker_base",
        [raw_slot_x, slot_y, amr_tray_z + 0.128],
        [0.072, 0.072, 0.004],
        COLORS["white"],
        mat_white,
    )
    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/RawPartMarker/Q1",
        "raw_part_marker_q1",
        [raw_slot_x - 0.022, slot_y - 0.022, amr_tray_z + 0.132],
        [0.025, 0.025, 0.003],
        COLORS["black"],
        mat_black,
    )
    fixed_box(
        f"{CELL_ROOT}/AMR/Mockup/RawPartMarker/Q2",
        "raw_part_marker_q2",
        [raw_slot_x + 0.022, slot_y + 0.022, amr_tray_z + 0.132],
        [0.025, 0.025, 0.003],
        COLORS["black"],
        mat_black,
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
        stage, f"{CELL_ROOT}/AMR/Mockup/RawPart", "sf_twin:role", "raw_workpiece"
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
