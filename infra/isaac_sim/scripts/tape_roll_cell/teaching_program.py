"""Sparse pendant points in equipment frames (metres, fixed-axis XYZ radians).

TCP +Z points from the mounting flange towards the finger tips: the captured
roll centre is TCP [0, 0, -insertion_depth]. No robot IK runs in this module.
"""
from copy import deepcopy
import math
import numpy as np

MOTIONS = ('tape_approach', 'tape_insert', 'tape_transfer', 'tape_retreat')


def _vector(value, size, label):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{label} requires {size} finite numbers')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f'{label} requires {size} finite numbers')
    return [float(v) for v in value]


def pose_matrix(pose):
    x,y,z,r,p,w = pose
    cr,sr,cp,sp,cw,sw = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(w),math.sin(w)
    matrix = np.eye(4)
    matrix[:3,:3] = [[cw*cp,cw*sp*sr-sw*cr,cw*sp*cr+sw*sr],
                      [sw*cp,sw*sp*sr+cw*cr,sw*sp*cr-cw*sr],[-sp,cp*sr,cp*cr]]
    matrix[:3,3] = [x,y,z]
    return matrix


def matrix_pose(matrix):
    r = matrix[:3,:3]
    pitch = math.atan2(-r[2,0], math.hypot(r[0,0],r[1,0]))
    roll,yaw = (math.atan2(r[2,1],r[2,2]), math.atan2(r[1,0],r[0,0])) if abs(math.cos(pitch)) > 1e-9 else (0.,math.atan2(-r[0,1],r[1,1]))
    return [*matrix[:3,3].tolist(),roll,pitch,yaw]


def matrix_pose7(matrix):
    # Branch on the largest quaternion component for stable half-turns.
    r=matrix[:3,:3];q=np.empty(4)
    candidates=[1+r[0,0]-r[1,1]-r[2,2],1-r[0,0]+r[1,1]-r[2,2],1-r[0,0]-r[1,1]+r[2,2],1+np.trace(r)]
    index=int(np.argmax(candidates));q[index]=math.sqrt(max(0.,candidates[index]))/2
    d=4*q[index]
    if index==3:q[:3]=[(r[2,1]-r[1,2])/d,(r[0,2]-r[2,0])/d,(r[1,0]-r[0,1])/d]
    else:
        i=index;j=(i+1)%3;k=(i+2)%3
        q[j]=(r[j,i]+r[i,j])/d;q[k]=(r[k,i]+r[i,k])/d;q[3]=(r[k,j]-r[j,k])/d
    return [*matrix[:3,3].tolist(),*(q/np.linalg.norm(q)).tolist()]


def pose7_matrix(pose):
    values=_vector(pose,7,'TCP pose');x,y,z,w=values[3:];norm=math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-12:raise ValueError('TCP quaternion must be nonzero')
    x,y,z,w=(v/norm for v in (x,y,z,w));m=np.eye(4)
    m[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
              [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
              [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    m[:3,3]=values[:3];return m


def base_matrix(world_to_base):
    pose=[world_to_base.get(k,0.) for k in ('x','y','z','roll','pitch','yaw')] if isinstance(world_to_base,dict) else world_to_base
    return pose_matrix(_vector(pose,6,'world_to_base'))


def validate_program(program):
    if not isinstance(program,dict) or type(program.get('schema_version')) is not int or program.get('schema_version') != 1:
        raise ValueError('Teaching program requires schema_version 1')
    result=deepcopy(program)
    for section in ('frames','points','sequences'):
        if not isinstance(result.get(section),dict):raise ValueError(f'Teaching program requires {section}')
    if set(result['frames']) != {'infeed','outfeed'}:raise ValueError('Equipment frames must be infeed and outfeed')
    for name,frame in result['frames'].items():
        if not isinstance(frame,dict):raise ValueError(f'Invalid frame {name}')
        frame['pose']=_vector(frame.get('pose'),6,name)
    for name,point in result['points'].items():
        if not isinstance(name,str) or not name or not isinstance(point,dict):raise ValueError('Invalid point name or definition')
        if point.get('frame') not in ('infeed','outfeed','base'):raise ValueError(f'Unknown point frame: {name}')
        point['pose']=_vector(point.get('pose'),6,name)
        if point.get('mode') not in ('joint','linear'):raise ValueError(f'Invalid interpolation mode: {name}')
        for field in ('velocity_scale','acceleration_scale'):
            value=point.get(field)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=1:
                raise ValueError(f'{name}.{field} must be in (0, 1]')
    if not set(MOTIONS)<=set(result['sequences']):raise ValueError('Teaching program requires all four task sequences')
    for name,sequence in result['sequences'].items():
        if not isinstance(name,str) or not isinstance(sequence,list) or not sequence or any(not isinstance(p,str) or p not in result['points'] for p in sequence):
            raise ValueError(f'Invalid point sequence: {name}')
    return result


def _point_world(program, name, world_to_base):
    point=program['points'][name]
    frame=base_matrix(world_to_base) if point['frame']=='base' else pose_matrix(program['frames'][point['frame']]['pose'])
    return frame @ pose_matrix(point['pose'])


def resolve_point(program,name,world_to_base):
    program=validate_program(program)
    return matrix_pose7(np.linalg.inv(base_matrix(world_to_base)) @ _point_world(program,name,world_to_base))


def capture_point(program,name,base_pose7,world_to_base):
    program=validate_program(program);point=program['points'][name]
    world=base_matrix(world_to_base) @ pose7_matrix(base_pose7)
    frame=base_matrix(world_to_base) if point['frame']=='base' else pose_matrix(program['frames'][point['frame']]['pose'])
    point['pose']=matrix_pose(np.linalg.inv(frame) @ world)
    return validate_program(program)


def resolve_program(program,world_to_base):
    program=validate_program(program);result={}
    for name,sequence in program['sequences'].items():
        result[name]=dict(cartesian_waypoints=[dict(pose=resolve_point(program,p,world_to_base),mode=program['points'][p]['mode']) for p in sequence],
            velocity_scale=min(program['points'][p]['velocity_scale'] for p in sequence),
            acceleration_scale=min(program['points'][p]['acceleration_scale'] for p in sequence))
    return result


def default_program(configuration):
    layout=configuration['cell']['tape_roll'];place=layout['place'];sx,sy,sz=layout['supply_center_m']
    px,py=place['first_center_xy_m'];pz=place['surface_z_m'];width=layout['material']['width_m'];depth=layout['insertion_depth_m']
    frames=dict(infeed=dict(pose=[sx,sy,sz,0.,0.,0.]),outfeed=dict(pose=[px,py,pz,0.,0.,0.]))
    points={}
    def point(name,frame,xyz,rpy,mode='linear'):
        points[name]=dict(frame=frame,pose=[*xyz,*rpy],mode=mode,velocity_scale=.1,acceleration_scale=.1)
    # TCP +Z = +X points from the robot through the horizontal bore. The
    # distal TCP extends depth beyond the ring centre at insertion.
    horizontal=[0.,math.pi/2,0.];down=[math.pi,0.,place.get('yaw_rad',0.)]
    clearance=max(.08,layout['material']['outer_radius_m']*2)
    lift=layout['extract_lift_m'];height=place['transport_height_m']
    point('approach','infeed',[depth-clearance,0.,0.],horizontal,'joint')
    point('insert','infeed',[depth,0.,0.],horizontal)
    point('lift','infeed',[depth,0.,lift],horizontal)
    point('extract','infeed',[depth-clearance,0.,lift],horizontal)
    point('carry','infeed',[depth-clearance,0.,height-sz],horizontal)
    point('turn','outfeed',[0.,0.,height-pz],down,'joint')
    place_z=width/2+layout['release_clearance_m']-depth
    point('place_approach','outfeed',[0.,0.,place_z+place['approach_clearance_m']],down)
    point('place','outfeed',[0.,0.,place_z],down)
    point('retreat','outfeed',[0.,0.,place_z+place['approach_clearance_m']],down)
    return validate_program(dict(schema_version=1,frames=frames,points=points,sequences=dict(
        tape_approach=['approach'],tape_insert=['insert'],tape_transfer=['lift','extract','carry','turn','place_approach','place'],tape_retreat=['retreat'])))


def equipment_transform(configuration,name):
    layout=configuration['cell']['tape_roll'];program=configuration['cell']['teaching_program']
    return pose_matrix(program['frames'][name]['pose']) @ np.linalg.inv(pose_matrix(layout['equipment_reference_frames'][name]['pose']))


def apply_layout(configuration,program):
    """Apply equipment frames to fixture/receipt geometry without robot solving."""
    from tape_roll_cell.fixtures import static_fixtures
    from tape_roll_cell.material import TapeRoll
    cell=configuration['cell'];layout=cell['tape_roll'];program=validate_program(program)
    layout.setdefault('equipment_reference_frames',deepcopy(default_program(configuration)['frames']))
    cell['teaching_program']=program
    layout['equipment_transforms']={name:equipment_transform(configuration,name).tolist() for name in ('infeed','outfeed')}
    roll=TapeRoll(**layout['material'])
    layout['static_boxes']=static_fixtures(layout,roll,cell['environment'],cell['conveyor'])
    source=[*layout['supply_center_m'],1.];place=layout['place'];receiver=[*place['first_center_xy_m'],place['surface_z_m']+roll.width_m/2,1.]
    for slot in layout.get('slots',[]):
        slot['supply_center_m']=(equipment_transform(configuration,'infeed') @ source)[:3].tolist()
        slot['receiver_center_m']=(equipment_transform(configuration,'outfeed') @ receiver)[:3].tolist()
    return configuration


def author_equipment_frames(stage,program,configuration=None):
    """Author stopped equipment poses; children use the canonical layout datum."""
    from pxr import Gf,UsdGeom
    program=validate_program(program)
    for name in ('infeed','outfeed'):
        matrix=equipment_transform(configuration,name) if configuration is not None else pose_matrix(program['frames'][name]['pose'])
        xform=UsdGeom.Xform.Define(stage,'/World/TapeRollCell/'+name.capitalize())
        xform.MakeMatrixXform().Set(Gf.Matrix4d(*matrix.T.flatten().tolist()))


def base_pose_to_frame(program,frame,base_pose7,world_to_base):
    anchor=base_matrix(world_to_base) if frame=='base' else pose_matrix(program['frames'][frame]['pose'])
    return matrix_pose(np.linalg.inv(anchor) @ base_matrix(world_to_base) @ pose7_matrix(base_pose7))


def frame_pose_to_base(program,frame,pose6,world_to_base):
    anchor=base_matrix(world_to_base) if frame=='base' else pose_matrix(program['frames'][frame]['pose'])
    return matrix_pose7(np.linalg.inv(base_matrix(world_to_base)) @ anchor @ pose_matrix(pose6))
