"""Environment, safety enclosure, CNC, lighting, and material construction."""


def build_environment(context):
    stage = context.stage
    CELL_ROOT = context.cell_root
    COLORS = context.colors
    Gf = context.gf
    UsdGeom = context.usd_geom
    UsdLux = context.usd_lux
    create_preview_material = context.create_preview_material
    static_box = context.static_box
    fixed_box = context.fixed_box
    create_sphere = context.create_sphere
    create_cylinder = context.create_cylinder
    set_custom_data = context.set_custom_data
    # -----------------------------------------------------------------------------
    # Lighting
    # -----------------------------------------------------------------------------
    dome_path = "/World/DomeLight"
    if not stage.GetPrimAtPath(dome_path).IsValid():
        dome = UsdLux.DomeLight.Define(stage, dome_path)
    else:
        dome = UsdLux.DomeLight(stage.GetPrimAtPath(dome_path))
    dome.CreateIntensityAttr(390.0)
    dome.CreateColorAttr(Gf.Vec3f(0.82, 0.86, 0.95))

    dist_path = "/World/DistantLight"
    if not stage.GetPrimAtPath(dist_path).IsValid():
        dist = UsdLux.DistantLight.Define(stage, dist_path)
    else:
        dist = UsdLux.DistantLight(stage.GetPrimAtPath(dist_path))
    dist.CreateIntensityAttr(930.0)
    dist.CreateAngleAttr(3.0)

    dist_xf = UsdGeom.Xformable(dist)
    dist_xf.ClearXformOpOrder()
    dist_xf.AddRotateXYZOp().Set(Gf.Vec3f(-42.0, 25.0, 12.0))

    rect = UsdLux.RectLight.Define(stage, f"{CELL_ROOT}/Environment/AreaLight")
    rect.CreateIntensityAttr(1900.0)
    rect.CreateWidthAttr(2.4)
    rect.CreateHeightAttr(1.8)
    rect.CreateColorAttr(Gf.Vec3f(0.95, 0.96, 1.0))
    rect_xf = UsdGeom.Xformable(rect)
    rect_xf.ClearXformOpOrder()
    rect_xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(0.30, -0.05, 2.80)
    )
    rect_xf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))

    # -----------------------------------------------------------------------------
    # Materials
    # -----------------------------------------------------------------------------
    mat_floor = create_preview_material(stage, "Floor", COLORS["floor"], roughness=0.72)
    mat_cell_floor = create_preview_material(
        stage, "CellFloor", COLORS["cell_floor"], roughness=0.62
    )
    mat_frame = create_preview_material(
        stage, "Frame", COLORS["frame"], roughness=0.28, metallic=0.35
    )
    mat_steel = create_preview_material(
        stage, "Steel", COLORS["steel"], roughness=0.30, metallic=0.58
    )
    mat_steel_dark = create_preview_material(
        stage, "SteelDark", COLORS["steel_dark"], roughness=0.34, metallic=0.45
    )
    mat_safety = create_preview_material(
        stage, "SafetyAmber", COLORS["safety"], roughness=0.38
    )
    mat_vision = create_preview_material(
        stage, "VisionNavy", COLORS["vision"], roughness=0.42
    )
    mat_kraft = create_preview_material(stage, "Kraft", COLORS["kraft"], roughness=0.75)
    mat_raw = create_preview_material(stage, "RawSlot", COLORS["raw"], roughness=0.62)
    mat_finished = create_preview_material(
        stage, "FinishedSlot", COLORS["finished"], roughness=0.45
    )
    mat_acrylic = create_preview_material(
        stage, "SmokeAcrylic", COLORS["acrylic"], roughness=0.12, opacity=0.30
    )
    mat_black = create_preview_material(stage, "Black", COLORS["black"], roughness=0.36)
    mat_white = create_preview_material(stage, "White", COLORS["white"], roughness=0.45)
    mat_red = create_preview_material(stage, "Red", COLORS["red"], roughness=0.30)
    mat_amber = create_preview_material(stage, "Amber", COLORS["amber"], roughness=0.25)
    mat_green = create_preview_material(stage, "Green", COLORS["green"], roughness=0.25)
    mat_blue = create_preview_material(stage, "Blue", COLORS["blue"], roughness=0.25)
    mat_cyan = create_preview_material(stage, "Cyan", COLORS["cyan"], roughness=0.25)

    # -----------------------------------------------------------------------------
    # 1. Factory floor + cell / AMR markings
    # -----------------------------------------------------------------------------
    static_box(
        "factory_floor",
        f"{CELL_ROOT}/Environment/FactoryFloor",
        "factory_floor",
        COLORS["floor"],
        mat_floor,
    )

    fixed_box(
        f"{CELL_ROOT}/Environment/CellSlab",
        "cell_slab",
        [0.25, 0.05, -0.003],
        [2.90, 2.55, 0.012],
        COLORS["cell_floor"],
        mat_cell_floor,
    )

    for name, pos, scale in [
        ("Left", [-1.18, 0.10, 0.003], [0.04, 2.35, 0.004]),
        ("Right", [1.68, 0.10, 0.003], [0.04, 2.35, 0.004]),
        ("Rear", [0.25, 1.29, 0.003], [2.90, 0.04, 0.004]),
    ]:
        fixed_box(
            f"{CELL_ROOT}/Safety/FloorMarking/{name}",
            f"floor_mark_{name.lower()}",
            pos,
            scale,
            COLORS["safety"],
            mat_safety,
        )

    for name, pos, scale in [
        ("DockLeft", [-0.47, -0.85, 0.003], [0.04, 0.92, 0.004]),
        ("DockRight", [0.87, -0.85, 0.003], [0.04, 0.92, 0.004]),
        ("DockRear", [0.20, -0.40, 0.003], [1.38, 0.04, 0.004]),
        ("DockFront", [0.20, -1.30, 0.003], [1.38, 0.04, 0.004]),
    ]:
        fixed_box(
            f"{CELL_ROOT}/AMR/DockMarking/{name}",
            f"dock_{name.lower()}",
            pos,
            scale,
            COLORS["safety"],
            mat_safety,
        )

    fixed_box(
        f"{CELL_ROOT}/AMR/DockMarking/PickupZone",
        "amr_pickup_zone",
        [0.20, -0.58, 0.004],
        [0.78, 0.34, 0.004],
        COLORS["vision"],
        mat_vision,
    )

    # -----------------------------------------------------------------------------
    # 2. Safety enclosure: 3-sided fence + open AMR interface
    # -----------------------------------------------------------------------------
    post_z = 0.80
    post_h = 1.60

    for name, pos in [
        ("RearLeft", [-1.13, 1.24, post_z]),
        ("RearRight", [1.63, 1.24, post_z]),
        ("FrontLeft", [-1.13, -0.96, post_z]),
        ("FrontRight", [1.63, -0.96, post_z]),
    ]:
        fixed_box(
            f"{CELL_ROOT}/Safety/Fence/{name}",
            name,
            pos,
            [0.055, 0.055, post_h],
            COLORS["charcoal"],
            mat_frame,
        )

    for name, pos, scale in [
        ("RearPanel", [0.25, 1.24, 0.80], [2.70, 0.018, 1.45]),
        ("LeftPanel", [-1.13, 0.15, 0.80], [0.018, 2.10, 1.45]),
        ("RightPanel", [1.63, 0.15, 0.80], [0.018, 2.10, 1.45]),
    ]:
        fixed_box(
            f"{CELL_ROOT}/Safety/Fence/{name}",
            name,
            pos,
            scale,
            COLORS["acrylic"],
            mat_acrylic,
        )

    for name, pos, scale in [
        ("RearTopRail", [0.25, 1.24, 1.53], [2.78, 0.045, 0.045]),
        ("LeftTopRail", [-1.13, 0.14, 1.53], [0.045, 2.20, 0.045]),
        ("RightTopRail", [1.63, 0.14, 1.53], [0.045, 2.20, 0.045]),
    ]:
        fixed_box(
            f"{CELL_ROOT}/Safety/Fence/{name}",
            name,
            pos,
            scale,
            COLORS["charcoal"],
            mat_frame,
        )

    for name, x in [("Left", -0.57), ("Right", 0.97)]:
        fixed_box(
            f"{CELL_ROOT}/Safety/AMRInterface/{name}Post",
            f"amr_interface_{name.lower()}",
            [x, -0.38, 0.66],
            [0.075, 0.075, 1.20],
            COLORS["charcoal"],
            mat_frame,
        )
        create_sphere(
            stage,
            f"{CELL_ROOT}/Safety/AMRInterface/{name}LED",
            0.022,
            [x, -0.43, 1.18],
            COLORS["green"],
            mat_green,
        )

    fixed_box(
        f"{CELL_ROOT}/Safety/AMRInterface/BeamIndicator",
        "amr_light_curtain_indicator",
        [0.20, -0.395, 0.52],
        [1.42, 0.012, 0.012],
        COLORS["cyan"],
        mat_cyan,
    )
    set_custom_data(
        stage,
        f"{CELL_ROOT}/Safety/AMRInterface",
        "sf_twin:role",
        "amr_logistics_safety_interface",
    )

    # -----------------------------------------------------------------------------
    # 3. CNC mockup - rear machine side
    # -----------------------------------------------------------------------------
    cnc_x, cnc_y = 0.82, 0.72

    static_box(
        "cnc_base",
        f"{CELL_ROOT}/CNC/Base",
        "cnc_base",
        COLORS["steel_dark"],
        mat_steel_dark,
    )

    static_box(
        "cnc_upper_housing",
        f"{CELL_ROOT}/CNC/UpperHousing",
        "cnc_upper",
        COLORS["steel"],
        mat_steel,
    )

    fixed_box(
        f"{CELL_ROOT}/CNC/FrontFrameTop",
        "cnc_front_top",
        [cnc_x, cnc_y - 0.415, 1.22],
        [1.08, 0.07, 0.13],
        COLORS["charcoal"],
        mat_frame,
    )
    fixed_box(
        f"{CELL_ROOT}/CNC/FrontFrameLeft",
        "cnc_front_left",
        [cnc_x - 0.52, cnc_y - 0.415, 0.92],
        [0.08, 0.07, 0.66],
        COLORS["charcoal"],
        mat_frame,
    )
    fixed_box(
        f"{CELL_ROOT}/CNC/FrontFrameRight",
        "cnc_front_right",
        [cnc_x + 0.52, cnc_y - 0.415, 0.92],
        [0.08, 0.07, 0.66],
        COLORS["charcoal"],
        mat_frame,
    )

    fixed_box(
        f"{CELL_ROOT}/CNC/WorkChamber",
        "cnc_chamber",
        [cnc_x, cnc_y - 0.38, 0.90],
        [0.90, 0.05, 0.48],
        COLORS["black"],
        mat_black,
    )

    fixed_box(
        f"{CELL_ROOT}/CNC/Door/Panel",
        "cnc_door",
        [cnc_x + 0.59, cnc_y - 0.47, 0.92],
        [0.48, 0.08, 0.66],
        COLORS["steel"],
        mat_steel,
    )
    fixed_box(
        f"{CELL_ROOT}/CNC/Door/Window",
        "cnc_window",
        [cnc_x + 0.59, cnc_y - 0.515, 1.02],
        [0.35, 0.018, 0.30],
        COLORS["acrylic"],
        mat_acrylic,
    )
    fixed_box(
        f"{CELL_ROOT}/CNC/Door/Handle",
        "cnc_handle",
        [cnc_x + 0.39, cnc_y - 0.575, 0.92],
        [0.035, 0.03, 0.22],
        COLORS["black"],
        mat_black,
    )
    set_custom_data(stage, f"{CELL_ROOT}/CNC/Door/Panel", "sf_twin:state", "open")

    fixed_box(
        f"{CELL_ROOT}/CNC/WorkArea/MachineTable",
        "cnc_machine_table",
        [cnc_x, cnc_y - 0.32, 0.69],
        [0.78, 0.34, 0.10],
        COLORS["steel"],
        mat_steel,
    )

    fixed_box(
        f"{CELL_ROOT}/CNC/WorkArea/Fixture",
        "cnc_fixture",
        [cnc_x - 0.08, cnc_y - 0.49, 0.79],
        [0.31, 0.20, 0.08],
        COLORS["frame"],
        mat_frame,
    )

    create_cylinder(
        stage,
        f"{CELL_ROOT}/CNC/WorkArea/Chuck",
        0.105,
        0.10,
        [cnc_x - 0.07, cnc_y - 0.53, 0.84],
        COLORS["steel_dark"],
        mat_steel_dark,
        axis="Y",
    )

    fixed_box(
        f"{CELL_ROOT}/CNC/ControlPanel/Body",
        "cnc_control",
        [cnc_x + 0.76, cnc_y - 0.34, 1.05],
        [0.28, 0.18, 0.52],
        COLORS["charcoal"],
        mat_frame,
    )
    fixed_box(
        f"{CELL_ROOT}/CNC/ControlPanel/Screen",
        "cnc_screen",
        [cnc_x + 0.76, cnc_y - 0.435, 1.13],
        [0.20, 0.018, 0.17],
        COLORS["blue"],
        mat_blue,
    )
    create_sphere(
        stage,
        f"{CELL_ROOT}/CNC/ControlPanel/CycleLED",
        0.018,
        [cnc_x + 0.72, cnc_y - 0.445, 0.94],
        COLORS["green"],
        mat_green,
    )
    create_sphere(
        stage,
        f"{CELL_ROOT}/CNC/ControlPanel/AlarmLED",
        0.018,
        [cnc_x + 0.80, cnc_y - 0.445, 0.94],
        COLORS["red"],
        mat_red,
    )

    for p, role in [
        (f"{CELL_ROOT}/CNC", "cnc_machine"),
        (f"{CELL_ROOT}/CNC/Door/Panel", "cnc_door"),
        (f"{CELL_ROOT}/CNC/WorkArea/Fixture", "cnc_fixture"),
    ]:
        set_custom_data(stage, p, "sf_twin:role", role)

    context.materials = {
        name: value for name, value in locals().items() if name.startswith("mat_")
    }
    context.cnc_x = cnc_x
    context.cnc_y = cnc_y
