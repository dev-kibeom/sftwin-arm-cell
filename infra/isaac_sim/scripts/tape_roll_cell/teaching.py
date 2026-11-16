"""Cell Cartesian teaching solved with the selected assembly's full URDF chain."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from internal_grasp_geometry import origin_transform


class TeachingSolver:
    def __init__(self, urdf, arm_names):
        self.urdf, self.names = urdf, arm_names
        limits = [urdf.find(f"joint[@name='{name}']/limit") for name in arm_names]
        by_child = {joint.find('child').get('link'): joint for joint in urdf.findall('joint')}
        chain, cursor = [], 'sf_grasp_tcp'
        while cursor in by_child:
            joint = by_child[cursor]
            axis = joint.find('axis')
            axis = np.array(list(map(float, axis.get('xyz', '1 0 0').split()))) if axis is not None else np.array([1., 0, 0])
            name = joint.get('name')
            moving = joint.get('type') != 'fixed'
            if moving and (name not in arm_names or joint.get('type') not in ('revolute', 'continuous')):
                raise ValueError('Cell teaching requires the selected revolute arm chain')
            chain.append((origin_transform(joint.find('origin')), axis/np.linalg.norm(axis),
                          arm_names.index(name) if moving else None))
            cursor = joint.find('parent').get('link')
        self.chain = list(reversed(chain))
        self.bounds = ([float(limit.get('lower')) for limit in limits],
                       [float(limit.get('upper')) for limit in limits])

    def fk(self, q):
        result = np.eye(4)
        for origin, axis, index in self.chain:
            result = result @ origin
            if index is not None:
                motion = np.eye(4)
                motion[:3, :3] = Rotation.from_rotvec(axis*q[index]).as_matrix()
                result = result @ motion
        return result

    def solve(self, target, seed):
        def error(q):
            actual = self.fk(q)
            return np.r_[actual[:3, 3]-target[:3, 3],
                         Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).as_rotvec()]
        solution = least_squares(error, seed, bounds=self.bounds, ftol=1e-12, xtol=1e-12, gtol=1e-12)
        if not solution.success or np.max(np.abs(error(solution.x))) > 1e-6:
            raise ValueError('Unreachable Cell Cartesian teaching')
        return solution.x.tolist()

    def path(self, start, finish):
        a, b = self.fk(start), self.fk(finish)
        rotation = Slerp([0, 1], Rotation.from_matrix(np.array([a[:3, :3], b[:3, :3]])))
        # At most 2 mm Cartesian knots. Check interpolated joint midpoints too;
        # endpoints alone cannot establish straight axial insertion/extraction.
        segments = max(1, int(np.ceil(np.linalg.norm(b[:3, 3]-a[:3, 3])/.002)))
        knots = [list(start)]
        for fraction in np.linspace(0, 1, segments+1)[1:]:
            target = np.eye(4)
            target[:3, 3] = a[:3, 3]+fraction*(b[:3, 3]-a[:3, 3])
            target[:3, :3] = rotation(fraction).as_matrix()
            knots.append(self.solve(target, knots[-1]))
        for index, (left, right) in enumerate(zip(knots, knots[1:])):
            actual = self.fk((np.array(left)+right)/2)
            fraction = (index+.5)/segments
            expected = a[:3, 3]+fraction*(b[:3, 3]-a[:3, 3])
            angle = Rotation.from_matrix(rotation(fraction).as_matrix().T @ actual[:3, :3]).magnitude()
            if np.linalg.norm(actual[:3, 3]-expected) > 5e-5 or angle > 5e-5:
                raise ValueError('Cell Cartesian trajectory interpolation error exceeds 50 um / 50 urad')
        return knots


def translated_teaching(solver, reference, supply_offset, receiver_offset, extract_lift_m):
    result = {}
    reference = dict(reference, lift=reference['insert'])
    orientation = solver.fk(reference['insert'])[:3, :3]
    for name, seed in reference.items():
        target = solver.fk(seed).copy()
        # Every slot extracts fully past the FRONT of the entire cartridge.
        if name in ('approach', 'insert', 'lift'):
            target[:3, 3] += supply_offset
        elif name != 'extract':
            target[:3, 3] += receiver_offset
        if name in ('lift', 'extract'):
            target[2, 3] += extract_lift_m
        target[:3, :3] = orientation
        result[name] = solver.solve(target, seed)
    return result


def trajectories(solver, teaching, previous=None):
    starts = dict(approach=teaching['approach'], insert=teaching['approach'],
                  lift=teaching['insert'], extract=teaching['lift'],
                  transfer_clearance=teaching['extract'], transfer=teaching['transfer_clearance'],
                  retreat=teaching['transfer'])
    if previous is not None:
        for name, seed in (('return_clearance', previous['retreat']),
                           ('supply_clearance', teaching['approach'])):
            target = solver.fk(seed).copy()
            target[2, 3] = max(solver.fk(teaching['transfer_clearance'])[2, 3], target[2, 3])
            teaching[name] = solver.solve(target, seed)
        starts.update(return_clearance=previous['retreat'],
                      supply_clearance=teaching['return_clearance'], approach=teaching['supply_clearance'])
    return {name: solver.path(start, teaching[name]) for name, start in starts.items()}
