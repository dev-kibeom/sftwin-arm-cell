"""Tape Cell appearance and overview; no physics, inventory or motion ownership."""


def author_presentation(stage, layout, roll, root, view):
    from pxr import Gf, Sdf, UsdGeom, UsdShade, UsdLux

    path = root + '/Presentation'
    UsdGeom.Xform.Define(stage, path)
    palette = {
        'floor': ((.11, .125, .145), .85, 0.),
        'frame': ((.18, .21, .25), .45, .5),
        'steel': ((.48, .53, .58), .32, .7),
        'paper': ((.65, .43, .20), .85, 0.),
        'supply': ((.93, .36, .055), .5, .05),
        'receiver': ((.025, .43, .40), .65, .05),
        'belt': ((.045, .07, .08), .9, 0.),
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
        cube = UsdGeom.Cube.Define(stage, path+'/'+name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(materials[look])

    # Cover the SDK's tiled ground appearance without replacing its collider.
    floor = UsdGeom.Mesh.Define(stage, path+'/FloorFinish')
    floor.CreatePointsAttr([(-3.,-3.,.002),(3.,-3.,.002),(3.,3.,.002),(-3.,3.,.002)])
    floor.CreateFaceVertexCountsAttr([4])
    floor.CreateFaceVertexIndicesAttr([0,1,2,3])
    floor.CreateSubdivisionSchemeAttr('none')
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(materials['floor'])
    box('RobotBaseplate', (0.,0.,.004), (.30,.30,.008), 'steel')

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
                    (x+(column-(len(text)-1)/2)*.019+dx,y,z+dz),
                    (.012,.0015,.002) if horizontal else (.002,.0015,.012), 'white')

    for label, center_key, look in [('Cartridge','supply_center_m','supply'),
                                    ('Conveyor','receiver_center_m','receiver')]:
        centers = [slot[center_key] for slot in layout['slots']]
        left = min(p[0] for p in centers)-roll.width_m/2-.025
        right = max(p[0] for p in centers)+roll.width_m/2+.025
        x,y,z = (left+right)/2,centers[0][1],centers[0][2]
        width, depth = right-left,2*roll.outer_radius_m+.07
        top = z-roll.outer_radius_m-.012
        box(label+'/Deck', (x,y,top-.009), (width,depth,.018), 'frame')
        box(label+'/FrontRail', (x,y-depth/2,top-.012), (width,.012,.034), look)
        box(label+'/BackRail', (x,y+depth/2,top-.012), (width,.012,.034), look)
        leg_height = top-.045
        for side, dx in [('Left',-width/2+.012),('Right',width/2-.012)]:
            for end, dy in [('Front',-depth/2+.02),('Back',depth/2-.02)]:
                box(label+'/'+side+end+'Leg', (x+dx,y+dy,.025+leg_height/2),
                    (.016,.016,leg_height), 'frame')
                box(label+'/'+side+end+'Foot', (x+dx,y+dy,.013), (.035,.035,.022), 'steel')
        if label == 'Conveyor':
            box(label+'/Belt', (x,y,top+.001), (width-.018,depth-.028,.003), 'belt')
            for side, dx in [('Left',-width/2+.012),('Right',width/2-.012)]:
                roller = UsdGeom.Cylinder.Define(stage, path+'/'+label+'/'+side+'Roller')
                roller.CreateAxisAttr('Y')
                roller.CreateRadiusAttr(.012)
                roller.CreateHeightAttr(depth-.018)
                roller.AddTranslateOp().Set(Gf.Vec3d(x+dx,y,top-.012))
                UsdShade.MaterialBindingAPI.Apply(roller.GetPrim()).Bind(materials['steel'])
        for index, center in enumerate(centers):
            box(label+f'/Slot{index}Badge', (center[0],y-depth/2-.008,top+.018),
                (.029,.005,.034), look)
            digit(label+f'/Slot{index}Number', index+1, center[0],y-depth/2-.011,top+.018)

    camera = UsdGeom.Camera.Define(stage, path+'/OverviewCamera')
    camera.CreateFocalLengthAttr(35.)
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
