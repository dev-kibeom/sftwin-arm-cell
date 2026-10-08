"""Cell root metadata, status-hardware authoring, and integration metadata."""


def initialize_cell_metadata(context):
    stage = context.stage
    cell_root = context.cell_root
    ensure_xform = context.ensure_xform
    set_custom_data = context.set_custom_data
    ensure_xform(stage, cell_root)
    for child in [
        "Environment",
        "Safety",
        "RobotStation",
        "Vision",
        "CameraRig",
        "CNC",
        "AMR",
        "Status",
        "Materials",
    ]:
        ensure_xform(stage, f"{cell_root}/{child}")
    set_custom_data(stage, cell_root, "sf_twin:cell_id", "CELL-A01")
    set_custom_data(
        stage, cell_root, "sf_twin:process", "direct_amr_to_cnc_machine_tending"
    )
    set_custom_data(stage, cell_root, "sf_twin:version", "v4.2")


def build_status_hardware(context):
    stage = context.stage
    CELL_ROOT = context.cell_root
    COLORS = context.colors
    fixed_box = context.fixed_box
    create_cylinder = context.create_cylinder
    create_sphere = context.create_sphere
    set_custom_data = context.set_custom_data
    mat_frame = context.materials["mat_frame"]
    mat_safety = context.materials["mat_safety"]
    mat_red = context.materials["mat_red"]
    mat_amber = context.materials["mat_amber"]
    mat_green = context.materials["mat_green"]
    mat_black = context.materials["mat_black"]
    # -----------------------------------------------------------------------------
    # 7. Status hardware
    # -----------------------------------------------------------------------------
    fixed_box(
        f"{CELL_ROOT}/Status/EStop/Pedestal",
        "estop_pedestal",
        [-0.94, -0.74, 0.72],
        [0.11, 0.11, 1.15],
        COLORS["charcoal"],
        mat_frame,
    )
    fixed_box(
        f"{CELL_ROOT}/Status/EStop/Box",
        "estop_box",
        [-0.94, -0.74, 1.31],
        [0.18, 0.14, 0.16],
        COLORS["safety"],
        mat_safety,
    )
    create_cylinder(
        stage,
        f"{CELL_ROOT}/Status/EStop/Button",
        0.055,
        0.04,
        [-0.94, -0.815, 1.32],
        COLORS["red"],
        mat_red,
        axis="Y",
    )
    set_custom_data(
        stage,
        f"{CELL_ROOT}/Status/EStop/Button",
        "sf_twin:role",
        "emergency_stop",
    )

    fixed_box(
        f"{CELL_ROOT}/Status/Tower/Pole",
        "tower_pole",
        [1.45, 0.93, 1.34],
        [0.03, 0.03, 0.46],
        COLORS["charcoal"],
        mat_frame,
    )
    create_sphere(
        stage,
        f"{CELL_ROOT}/Status/Tower/Red",
        0.040,
        [1.45, 0.93, 1.60],
        COLORS["red"],
        mat_red,
    )
    create_sphere(
        stage,
        f"{CELL_ROOT}/Status/Tower/Amber",
        0.040,
        [1.45, 0.93, 1.51],
        COLORS["amber"],
        mat_amber,
    )
    create_sphere(
        stage,
        f"{CELL_ROOT}/Status/Tower/Green",
        0.040,
        [1.45, 0.93, 1.42],
        COLORS["green"],
        mat_green,
    )

    fixed_box(
        f"{CELL_ROOT}/Status/CellID",
        "cell_id_plate",
        [1.58, -0.62, 1.05],
        [0.012, 0.30, 0.22],
        COLORS["black"],
        mat_black,
    )
    set_custom_data(
        stage,
        f"{CELL_ROOT}/Status/CellID",
        "sf_twin:label",
        "SF-TWIN CELL-A01 / DIRECT AMR-CNC TENDING",
    )


def apply_integration_metadata(context):
    stage = context.stage
    CELL_ROOT = context.cell_root
    set_custom_data = context.set_custom_data
    set_vec3_attribute = context.set_vec3_attribute
    amr_x, amr_y = context.amr_x, context.amr_y
    raw_slot_x, done_slot_x, slot_y, amr_tray_z = (
        context.raw_slot_x,
        context.done_slot_x,
        context.slot_y,
        context.amr_tray_z,
    )
    cnc_x, cnc_y = context.cnc_x, context.cnc_y
    # -----------------------------------------------------------------------------
    # 8. Semantic / integration metadata
    # -----------------------------------------------------------------------------
    semantic_map = {
        f"{CELL_ROOT}/CNC": "machine_tool",
        f"{CELL_ROOT}/m0609": "manipulator",
        f"{CELL_ROOT}/Vision/Camera_Sensor": "vision_sensor",
        f"{CELL_ROOT}/AMR/Mockup": "mobile_robot_placeholder",
        f"{CELL_ROOT}/AMR/Mockup/Tray": "robot_pick_surface",
        f"{CELL_ROOT}/CNC/WorkArea/Fixture": "cnc_fixture",
        f"{CELL_ROOT}/Safety/AMRInterface": "logistics_safety_interface",
    }

    for path, role in semantic_map.items():
        set_custom_data(stage, path, "sf_twin:semantic_role", role)

    for key, val in {
        "sf_twin:amr_docked": True,
        "sf_twin:amr_pose_verified": False,
        "sf_twin:raw_part_detected": False,
        "sf_twin:cnc_machine_ready": True,
        "sf_twin:cnc_door_open": True,
        "sf_twin:cnc_cycle_complete": False,
        "sf_twin:amr_interface_clear": True,
        "sf_twin:robot_motion_permit": False,
    }.items():
        set_custom_data(stage, CELL_ROOT, key, val)

    set_vec3_attribute(
        stage,
        CELL_ROOT,
        "sf_twin:nominal_amr_dock_pose",
        [amr_x, amr_y, 0.0],
    )
    set_vec3_attribute(
        stage,
        CELL_ROOT,
        "sf_twin:nominal_raw_pick_pose",
        [raw_slot_x, slot_y, amr_tray_z + 0.12],
    )
    set_vec3_attribute(
        stage,
        CELL_ROOT,
        "sf_twin:nominal_cnc_place_pose",
        [cnc_x - 0.08, cnc_y - 0.49, 0.88],
    )
    set_vec3_attribute(
        stage,
        CELL_ROOT,
        "sf_twin:nominal_finished_place_pose",
        [done_slot_x, slot_y, amr_tray_z + 0.10],
    )
