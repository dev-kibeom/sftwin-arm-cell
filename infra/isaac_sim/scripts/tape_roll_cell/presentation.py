"""Tape Cell appearance and overview; no physics, inventory or motion ownership."""


def author_presentation(stage, layout, roll, root, view, environment):
    from pxr import Gf, Sdf, UsdGeom, UsdShade, UsdLux

    path = root + '/Presentation'
    UsdGeom.Xform.Define(stage, path)
    palette = {
        'floor': ((.012, .11, .035), .60, 0.),
        'frame': ((.18, .21, .25), .45, .5),
        'steel': ((.48, .53, .58), .32, .7),
        'paper': ((.65, .43, .20), .85, 0.),
        'supply': ((.93, .36, .055), .5, .05),
        'receiver': ((.025, .43, .40), .65, .05),
        'white': ((.85, .89, .90), .6, 0.),
    }
    materials = {}
    for name, (color, roughness, metallic) in palette.items():
        material = UsdShade.Material.Define(stage, path+'/Looks/'+name)
        shader = UsdShade.Shader.Define(stage, str(material.GetPath())+'/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        materials[name] = material

    def box(name, center, size, look):
        equipment='Infeed' if name.startswith('Infeed/') or name=='PickDatum' else 'Outfeed' if name.startswith('Outfeed/') or name=='PlaceDatum' else None
        target=root+'/'+equipment+'/Presentation/'+name if equipment else path+'/'+name
        cube = UsdGeom.Cube.Define(stage, target)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(materials[look])

    floor_z = environment['floor_z_m']
    support_z = environment['robot_support_top_z_m']
    sx,sy = environment['robot_support_size_m']
    box('RobotTopFlange', (0.,0.,support_z+.003), (sx+.025,sy+.025,.006), 'steel')
    box('RobotFootFlange', (0.,0.,floor_z+.012), (sx+.055,sy+.055,.024), 'steel')
    def digit(label, value, x, y, z):
        # Small physical-size slot numbers, with no font/texture dependency.
        segments = {'0':'abcedf', '1':'bc', '2':'abged', '3':'abgcd', '4':'fgbc',
                    '5':'afgcd', '6':'afgecd', '7':'abc', '8':'abcdefg', '9':'abfgcd'}
        poses = {'a':(0.,.012,True), 'g':(0.,0.,True), 'd':(0.,-.012,True),
                 'f':(-.006,.006,False), 'b':(.006,.006,False),
                 'e':(-.006,-.006,False), 'c':(.006,-.006,False)}
        text = str(value)
        for column, character in enumerate(text):
            for segment in segments[character]:
                dx,dz,horizontal = poses[segment]
                box(f'{label}/Digit{column}{segment}',
                    (x+((column-(len(text)-1)/2)*.019+dx)*.7,y,z+dz*.7),
                    (.0084,.0015,.0014) if horizontal else (.0014,.0015,.0084), 'white')

    def legs(label,cx,cy,span_x,span_y,surface):
        bottom=floor_z+.024;height=surface-.025-bottom
        for ix,dx in enumerate((-span_x/2+.025,span_x/2-.025)):
            for iy,dy in enumerate((-span_y/2+.025,span_y/2-.025)):
                box(f'{label}/Leg{ix}{iy}',(cx+dx,cy+dy,bottom+height/2),(.025,.025,height),'frame')
                box(f'{label}/Foot{ix}{iy}',(cx+dx,cy+dy,floor_z+.012),(.05,.05,.024),'steel')
    cfg=layout['conveyor_presentation']
    x=cfg['infeed_center_x_m'];y=layout['supply_center_m'][1]
    from tape_roll_cell.fixtures import cartridge_bed_bottom_z
    z=cartridge_bed_bottom_z(layout,roll)
    length=cfg['infeed_length_m'];width=2*roll.outer_radius_m+.055
    for sign in (-1,1):
        box(f'Infeed/Frame{int(sign>0)}',(x,y+sign*width/2,z-.022),(length,.025,.045),'supply')
    legs('Infeed',x,y,length,width,z)
    px,py=layout['place']['first_center_xy_m'];top=layout['place']['surface_z_m']
    length=cfg['outfeed_length_m'];cy=py-length/2+.09
    for sign in (-1,1):
        box(f'Outfeed/Frame{int(sign>0)}',(px+sign*(roll.outer_radius_m+.02),cy,top-.027),(.025,length,.05),'receiver')
    legs('Outfeed',px,cy,2*roll.outer_radius_m+.07,length,top)
    box('PickDatum',(layout['supply_center_m'][0],y-width/2-.014,z+.01),(.025,.008,.025),'white')
    box('PlaceDatum',(px,py+.1,top-.01),(.04,.006,.03),'receiver')

    camera = UsdGeom.Camera.Define(stage, path+'/OverviewCamera')
    camera.CreateFocalLengthAttr(24.)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.01,100.))
    transform = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*view['eye']), Gf.Vec3d(*view['target']),
                                       Gf.Vec3d(0.,0.,1.)).GetInverse()
    camera.MakeMatrixXform().Set(transform)
    light = UsdLux.RectLight.Define(stage, path+'/WorkLight')
    light.CreateWidthAttr(2.)
    light.CreateHeightAttr(2.)
    light.CreateIntensityAttr(1100.)
    light.AddTranslateOp().Set(Gf.Vec3d(.3,.2,2.5))
    return materials
