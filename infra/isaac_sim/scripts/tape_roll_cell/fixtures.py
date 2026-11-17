"""Cell-owned static geometry shared by PhysX and the planning scene."""
import math

ROOT = '/World/TapeRollCell'


def static_fixtures(layout, roll, environment):
    boxes = []

    def box(name, center, dimensions, angle=0.):
        boxes.append(dict(id=ROOT+'/'+name, geometry=dict(type='box', dimensions_m=dimensions),
            pose=dict(position_m=center, quaternion_xyzw=[0.,0.,math.sin(angle/2),math.cos(angle/2)])))

    floor = float(environment['floor_z_m'])
    thickness = float(environment['floor_thickness_m'])
    box('EpoxyFloor', [0.,0.,floor-thickness/2], [*environment['floor_size_m'],thickness])
    top = float(environment['robot_support_top_z_m'])
    box('RobotPedestal', [0.,0.,(floor+top)/2], [*environment['robot_support_size_m'],top-floor])
    for slot in layout['slots']:
        x,y,z = slot['supply_center_m']
        r,w = roll.outer_radius_m,roll.width_m
        prefix = f"Cartridge{slot['index']:03d}"
        box(prefix+'Floor', [x,y,z-r-.005], [w+.008,2*r+.024,.01])
        for side,sign in [('Left',-1),('Right',1)]:
            box(prefix+side+'Guide', [x,y+sign*(r+.009),z-r+.015], [w+.008,.01,.04])
        x,y,z = slot['receiver_center_m']
        # An open annular nest supports the outer rim. Fingers can contract and
        # retreat through the bore without crossing a solid plate.
        radius = (roll.inner_radius_m+roll.outer_radius_m)/2+.002
        radial = (roll.outer_radius_m-roll.inner_radius_m)*.5
        tangent = 2*radius*math.tan(math.pi/8)*.65
        support_top = z-roll.width_m/2
        for index in range(8):
            angle = 2*math.pi*index/8
            box(f"Receiver{slot['index']:03d}Pad{index}",
                [x+radius*math.cos(angle),y+radius*math.sin(angle),support_top-.006],
                [radial,tangent,.012],angle)
    return boxes
