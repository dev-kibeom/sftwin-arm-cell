"""URDF kinematics shared by Robot initialization and Tool geometry consumers."""
import math
import numpy as np


def origin_transform(origin):
    result = np.eye(4)
    if origin is None:
        return result
    x, y, z = map(float, origin.get('rpy', '0 0 0').split())
    cx, cy, cz, sx, sy, sz = math.cos(x), math.cos(y), math.cos(z), math.sin(x), math.sin(y), math.sin(z)
    result[:3, :3] = [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                       [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]]
    result[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
    return result


def link_transforms(urdf, positions):
    """URDF forward transforms, used for geometry and recorded teaching poses."""
    transforms = {}
    children = {j.find('child').get('link') for j in urdf.findall('joint')}
    transforms.update({link.get('name'): np.eye(4) for link in urdf.findall('link') if link.get('name') not in children})
    pending = list(urdf.findall('joint'))
    while pending:
        progress = False
        for joint in pending[:]:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in transforms:
                continue
            local = origin_transform(joint.find('origin'))
            motion = np.eye(4)
            q = positions.get(joint.get('name'), 0.)
            axis = joint.find('axis')
            axis = np.array(list(map(float, axis.get('xyz').split()))) if axis is not None else np.array([1., 0, 0])
            if joint.get('type') == 'prismatic':
                motion[:3, 3] = axis * q
            elif joint.get('type') in ('revolute', 'continuous'):
                axis = axis / np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                motion[:3, :3] = np.eye(3) + math.sin(q)*skew + (1-math.cos(q))*(skew @ skew)
            transforms[child] = transforms[parent] @ local @ motion
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError('Disconnected URDF')
    return transforms


