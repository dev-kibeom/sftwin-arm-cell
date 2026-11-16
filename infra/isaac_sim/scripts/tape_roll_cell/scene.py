"""Cell scene: dynamic hollow rolls, axial cartridge and passive conveyor receiver."""
from tape_roll_cell.material import TapeRoll


def build_scene(*, configuration, stage, **unused):
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, UsdShade
    config = configuration['cell']['tape_roll']
    roll = TapeRoll(**config['material'])
    root = '/World/TapeRollCell'
    if stage.GetPrimAtPath(root):
        stage.RemovePrim(root)
    UsdGeom.Xform.Define(stage, root)
    material = UsdShade.Material.Define(stage, root+'/ContactMaterial')
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(roll.static_friction)
    physics_material.CreateDynamicFrictionAttr(roll.dynamic_friction)
    physics_material.CreateRestitutionAttr(0.)

    def collision(prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        contact.CreateContactOffsetAttr(.0002)
        contact.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')

    roll_paths = []
    for slot in config['slots']:
        path = f'{root}/Roll{slot["index"]:03d}'
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(*slot['supply_center_m']))
        # Local annulus Z becomes horizontal world X; local Y remains world Y.
        body.AddRotateYOp().Set(90.)
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
        mass.CreateMassAttr(roll.mass_kg)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0,0,0))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*roll.inertia()))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(1.))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim())
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(8)
        PhysxSchema.PhysxContactReportAPI.Apply(body.GetPrim()).CreateThresholdAttr(0.)
        faces = [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
        for index, vertices in enumerate(roll.convex_segments()):
            mesh = UsdGeom.Mesh.Define(stage, f'{path}/Sector{index:03d}')
            mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in vertices])
            mesh.CreateFaceVertexCountsAttr([4]*6)
            mesh.CreateFaceVertexIndicesAttr([v for face in faces for v in face])
            mesh.CreateSubdivisionSchemeAttr('none')
            mesh.CreateDisplayColorAttr([Gf.Vec3f(.74,.55,.24)])
            collision(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        roll_paths.append(path)

    def box(name, center, dimensions):
        cube = UsdGeom.Cube.Define(stage, root+'/'+name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*dimensions))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.2,.3,.4)])
        collision(cube.GetPrim())

    # No axial cap: extraction and retreat can leave along -X. Side guides
    # constrain the outside diameter; the bore remains clear for all fingers.
    for slot in config['slots']:
        for label, center in ((f'Cartridge{slot["index"]:03d}',slot['supply_center_m']),
                              (f'Receiver{slot["index"]:03d}',slot['receiver_center_m'])):
            x,y,z = center
            r, width = roll.outer_radius_m, roll.width_m
            box(label+'Floor', (x,y,z-r-.005), (width+.008,2*r+.024,.01))
            for side, sign in (('Left',-1),('Right',1)):
                box(label+side+'Guide', (x,y+sign*(r+.009),z-r+.015),
                    (width+.008,.01,.04))
    from tape_roll_cell.supply import TapeSupply
    from tape_roll_cell.lifecycle import bind_stop_reset
    supply = TapeSupply(len(roll_paths))
    stop_subscription = bind_stop_reset(stage, roll_paths, supply)
    return dict(stop_subscription=stop_subscription, roll_path=roll_paths[0], roll_paths=roll_paths, root_path=root,
                roll=roll, supply=supply)
