"""Cell-owned static geometry shared by PhysX and the planning scene."""
import math
import numpy as np
from tape_roll_cell.teaching_program import matrix_pose7

ROOT = '/World/TapeRollCell'
CARTRIDGE_BED_THICKNESS_M = .02


def cartridge_bed_bottom_z(layout, roll):
    """Leave 35 mm below the roll for the V cradle, then the bed thickness."""
    return layout['supply_center_m'][2]-roll.outer_radius_m-.035-CARTRIDGE_BED_THICKNESS_M


def static_fixtures(layout, roll, environment, conveyor):
    boxes = []

    def box(name, center, dimensions, angle=0.):
        equipment='infeed' if name=='InfeedBelt' else 'outfeed' if name.startswith('Outfeed') else None
        identity=ROOT+('/'+equipment.capitalize() if equipment else '')+'/'+name
        local=dict(position_m=center,quaternion_xyzw=[0.,0.,math.sin(angle/2),math.cos(angle/2)])
        transform=np.array(layout.get('equipment_transforms',{}).get(equipment,np.eye(4)))
        rotation=np.eye(4);rotation[:3,3]=center
        rotation[:3,:3]=[[math.cos(angle),-math.sin(angle),0.],[math.sin(angle),math.cos(angle),0.],[0.,0.,1.]]
        world=matrix_pose7(transform @ rotation)
        boxes.append(dict(id=identity,geometry=dict(type='box',dimensions_m=dimensions),
            pose=dict(position_m=world[:3],quaternion_xyzw=world[3:]),local_pose=local,equipment=equipment))

    floor = float(environment['floor_z_m'])
    thickness = float(environment['floor_thickness_m'])
    box('EpoxyFloor', [0.,0.,floor-thickness/2], [*environment['floor_size_m'],thickness])
    top = float(environment['robot_support_top_z_m'])
    box('RobotPedestal', [0.,0.,(floor+top)/2], [*environment['robot_support_size_m'],top-floor])
    x,y,z = layout['supply_center_m']
    belt_top = cartridge_bed_bottom_z(layout,roll)
    box('InfeedBelt',[conveyor['infeed_center_x_m'],y,belt_top-.0125],
        [conveyor['infeed_length_m'],2*roll.outer_radius_m+.04,.025])
    px,py=layout['place']['first_center_xy_m'];top=layout['place']['surface_z_m']
    # Two narrow moving belts support the outer rim and leave the bore open.
    for name,sign in [('OutfeedLeft',-1),('OutfeedRight',1)]:
        box(name,[px+sign*(roll.outer_radius_m-.005),py-conveyor['outfeed_length_m']/2+.09,top-.01],
            [.008,conveyor['outfeed_length_m'],.02])
    return boxes
