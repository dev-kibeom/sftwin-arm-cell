"""Robot pedestal, mounting hardware, and M0609 reference construction."""

import os
from pathlib import Path

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
    robot_x, robot_y = 0.15, -0.15

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

    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not configured_root:
        raise RuntimeError(
            "SFTWIN_PROJECT_ROOT is not configured. Run 1_before_play.py "
            "before constructing the robot station."
        )
    project_root = Path(configured_root).expanduser().resolve()
    robot_usd = (
        project_root / "infra/isaac_sim/assets/usd/m0609_robotiq_2f85_generated.usd"
    )
    generated_urdf = (
        project_root / "infra/isaac_sim/assets/urdf/generated/m0609_robotiq_2f85.urdf"
    )
    source_xacro = (
        project_root / "infra/isaac_sim/assets/urdf/assemblies/m0609_robotiq_2f85.xacro"
    )
    provenance = robot_usd.with_suffix(".provenance.json")
    robot_path = f"{CELL_ROOT}/m0609"

    if robot_usd.exists():
        verify_current_artifact(source_xacro, generated_urdf, robot_usd, provenance)
        add_reference_to_stage(str(robot_usd), robot_path, prim_type="Xform")
        robot_prim = stage.GetPrimAtPath(robot_path)
        if robot_prim.IsValid():
            rx = UsdGeom.Xformable(robot_prim)
            rx.ClearXformOpOrder()
            rx.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(robot_x, robot_y, 0.68)
            )
            rx.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            robot_prim.SetCustomDataByKey("sf_twin:role", "manipulator")
            print(">>> [SUCCESS] Doosan M0609 loaded")
    else:
        print(f">>> [WARN] Robot USD not found: {robot_usd}")
        print(">>> Run 0_pre_build.py first.")

    context.robot_x = robot_x
    context.robot_y = robot_y
