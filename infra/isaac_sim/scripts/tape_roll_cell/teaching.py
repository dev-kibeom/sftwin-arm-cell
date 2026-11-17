"""Cell Cartesian teaching solved with the selected assembly's full URDF chain."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from gripper_runtime.radial_geometry import origin_transform


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
        angle = Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude()
        segments = max(1, int(np.ceil(np.linalg.norm(b[:3, 3]-a[:3, 3])/.002)),
                       int(np.ceil(angle/.05)))
        knots = [list(start)]
        for fraction in np.linspace(0, 1, segments+1)[1:]:
            target = np.eye(4)
            target[:3, 3] = a[:3, 3]+fraction*(b[:3, 3]-a[:3, 3])
            target[:3, :3] = rotation(fraction).as_matrix()
            knots.append(self.solve(target, knots[-1]))
        def refine(left, right, low, high, depth=0):
            fraction = (low+high)/2
            actual = self.fk((np.array(left)+right)/2)
            target = np.eye(4)
            target[:3,3] = a[:3,3]+fraction*(b[:3,3]-a[:3,3])
            target[:3,:3] = rotation(fraction).as_matrix()
            angle_error = Rotation.from_matrix(target[:3,:3].T @ actual[:3,:3]).magnitude()
            if np.linalg.norm(actual[:3,3]-target[:3,3]) <= 5e-5 and angle_error <= 5e-5:
                return [right]
            if depth >= 10:
                raise ValueError('Cell Cartesian trajectory interpolation exceeds 50 um / 50 urad')
            middle = self.solve(target,(np.array(left)+right)/2)
            return (refine(left,middle,low,fraction,depth+1)+
                    refine(middle,right,fraction,high,depth+1))
        refined = [knots[0]]
        for index,(left,right) in enumerate(zip(knots,knots[1:])):
            refined.extend(refine(left,right,index/segments,(index+1)/segments))
        return refined


def vertical_place_teaching(solver, reference, supply_center, receiver_center, layout, previous=None):
    depth = layout['insertion_depth_m']
    source = solver.fk(reference['insert'])
    original_center = (source @ np.array([0.,0.,-depth,1.]))[:3]
    delta = np.array(supply_center)-original_center
    taught = {}
    for name in ('approach','insert','extract'):
        target = solver.fk(reference[name]).copy()
        target[:3,3] += delta
        if name == 'extract':target[2,3] += layout['extract_lift_m']
        taught[name] = solver.solve(target,(previous or reference)[name])
    target = solver.fk(taught['insert']).copy()
    target[2,3] += layout['extract_lift_m']
    taught['lift'] = solver.solve(target,taught['insert'])
    clearance = layout['place']['transport_height_m']
    target = solver.fk(taught['extract']).copy();target[2,3] = clearance
    taught['carry_lift'] = solver.solve(target,taught['extract'])
    target[:3,:3] = Rotation.from_euler('x',np.pi).as_matrix()
    taught['turn'] = solver.solve(target,taught['carry_lift'])
    place = target.copy()
    place[:3,3] = np.array(receiver_center)+np.array([0.,0.,layout['release_clearance_m']-depth])
    target = place.copy();target[2,3] += layout['place']['approach_clearance_m']
    taught['transfer_clearance'] = solver.solve(target,taught['turn'])
    taught['transfer'] = solver.solve(place,taught['transfer_clearance'])
    taught['retreat'] = list(taught['transfer_clearance'])
    starts = dict(lift=taught['insert'],extract=taught['lift'],carry_lift=taught['extract'],
        turn=taught['carry_lift'],transfer_clearance=taught['turn'],transfer=taught['transfer_clearance'],
        retreat=taught['transfer'])
    if previous is None:
        target = solver.fk(taught['approach']).copy();target[2,3] = clearance
        entry = solver.solve(target,taught['approach'])
        starts['approach'] = entry
    else:
        target = solver.fk(previous['retreat']).copy();target[2,3] = clearance
        taught['return_clearance'] = solver.solve(target,previous['retreat'])
        target[:3,:3] = solver.fk(taught['approach'])[:3,:3]
        taught['return_turn'] = solver.solve(target,taught['return_clearance'])
        target = solver.fk(taught['approach']).copy();target[2,3] = clearance
        taught['supply_clearance'] = solver.solve(target,taught['return_turn'])
        starts.update(return_clearance=previous['retreat'],return_turn=taught['return_clearance'],
            supply_clearance=taught['return_turn'],approach=taught['supply_clearance'])
    paths = {name:solver.path(start,taught[name]) for name,start in starts.items()}
    paths['insert'] = solver.path(taught['approach'],taught['insert'])
    return taught,paths
