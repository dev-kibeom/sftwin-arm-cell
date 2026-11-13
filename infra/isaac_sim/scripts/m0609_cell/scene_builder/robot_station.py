"""Robot pedestal, mounting hardware, and M0609 reference construction."""

from scene_builder.model_provenance import verify_current_artifact


def build_robot_station(context):
    stage = context.stage
    CELL_ROOT = context.cell_root
    COLORS = context.colors
    Gf = context.gf
    UsdGeom = context.usd_geom
    static_box = context.static_box
    create_cylinder = context.create_cylinder
    add_reference_to_stage = context.add_reference_to_stage
    mat_frame = context.materials["mat_frame"]
    mat_steel = context.materials["mat_steel"]
    mat_black = context.materials["mat_black"]
    # -----------------------------------------------------------------------------
    # 4. Robot pedestal + Doosan M0609
    # -----------------------------------------------------------------------------
    selected = context.scene_configuration
    robot_x, robot_y, robot_z = selected.position_m

    static_box(
        "pedestal",
        f"{CELL_ROOT}/RobotStation/Pedestal",
        "robot_pedestal",
        COLORS["charcoal"],
        mat_frame,
    )
    static_box(
        "pedestal_top",
        f"{CELL_ROOT}/RobotStation/PedestalTop",
        "robot_pedestal_top",
        COLORS["steel"],
        mat_steel,
    )
    static_box(
        "robot_riser",
        f"{CELL_ROOT}/RobotStation/RobotRiser",
        "robot_riser",
        COLORS["black"],
        mat_black,
    )

    for i, (dx, dy) in enumerate(
        [
            (-0.09, -0.09),
            (-0.09, 0.09),
            (0.09, -0.09),
            (0.09, 0.09),
        ]
    ):
        create_cylinder(
            stage,
            f"{CELL_ROOT}/RobotStation/MountBolts/Bolt_{i + 1}",
            0.012,
            0.018,
            [robot_x + dx, robot_y + dy, 0.694],
            COLORS["steel"],
            mat_steel,
        )

    model = selected.model
    robot_usd = model.usd
    robot_path = selected.robot_prim_path

    if robot_usd.exists():
        verify_current_artifact(
            model.source, model.generated_urdf, robot_usd, model.provenance,
            xacro_arguments=model.xacro_arguments,
        )
        add_reference_to_stage(str(robot_usd), robot_path, prim_type="Xform")
        robot_prim = stage.GetPrimAtPath(robot_path)
        if robot_prim.IsValid():
            rx = UsdGeom.Xformable(robot_prim)
            rx.ClearXformOpOrder()
            rx.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(robot_x, robot_y, robot_z)
            )
            rx.AddRotateXYZOp().Set(Gf.Vec3f(*selected.rotation_deg))
            robot_prim.SetCustomDataByKey("sf_twin:role", "manipulator")
    else:
        print(
            "Simulation robot is unavailable because its generated USD is missing. "
            "Run 0_pre_build.py to build the robot asset."
        )

    context.robot_x = robot_x
    context.robot_y = robot_y
