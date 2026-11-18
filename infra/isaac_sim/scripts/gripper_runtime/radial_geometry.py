"""Selected URDF collision envelopes and explicit-bore solve in canonical TCP axes."""
import itertools
import math
from pathlib import Path
import struct

import numpy as np


from isaac_runtime.urdf_kinematics import origin_transform, link_transforms


def collision_triangles(collision):
    geometry = collision.find('geometry')
    mesh, box = geometry.find('mesh'), geometry.find('box')
    if mesh is not None:
        raw = Path(mesh.get('filename')).read_bytes()
        count = struct.unpack_from('<I', raw, 80)[0]
        if len(raw) != 84 + count*50:
            raise ValueError('Experiment requires binary STL collision mesh')
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        triangles = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['vertices'].astype(float)
        triangles *= np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
    elif box is not None:
        vertices = np.array(list(itertools.product((-1, 1), repeat=3))) * np.array(list(map(float, box.get('size').split()))) / 2
        faces = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                 [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
        triangles = vertices[faces]
    else:
        raise ValueError('Unsupported experiment collision geometry')
    return triangles


def slab_radius(triangles, lower, upper):
    """Clip triangle edges to an axial slab; include intersections, not only vertices."""
    points = []
    for triangle in triangles:
        polygon = list(triangle)
        for plane, sign in ((lower, 1), (upper, -1)):
            clipped = []
            for a, b in zip(polygon, polygon[1:] + polygon[:1]):
                inside_a, inside_b = sign*(a[2]-plane) >= -1e-10, sign*(b[2]-plane) >= -1e-10
                if inside_a:
                    clipped.append(a)
                if inside_a != inside_b:
                    clipped.append(a + (b-a)*((plane-a[2])/(b[2]-a[2])))
            polygon = clipped
        points.extend(polygon)
    if not points:
        raise ValueError('No finger collision surface at insertion depth')
    return float(np.max(np.linalg.norm(np.array(points)[:, :2], axis=1)))


class ToolGeometry:
    def __init__(self, urdf, settings):
        self.urdf, self.settings = urdf, settings
        primary = urdf.find(f"joint[@name='{settings['primary_joint']}']")
        self.base_link = primary.find('parent').get('link')
        self.linkage = settings['joint_model']
        moving = {urdf.find(f"joint[@name='{name}']/child").get('link') for name in self.linkage}
        while True:
            descendants = {j.find('child').get('link') for j in urdf.findall('joint')
                           if j.get('type') == 'fixed' and j.find('parent').get('link') in moving}
            if descendants <= moving:
                break
            moving |= descendants
        self.collisions = [(link.get('name'), collision, collision_triangles(collision))
                           for link in urdf.findall('link') if link.get('name') in moving
                           for collision in link.findall('collision')]
        if not self.collisions:
            raise ValueError('Selected fingers have no collision geometry')

    def triangles(self, position):
        targets = {name: position*j.get('mimic', {}).get('multiplier', 1.) + j.get('mimic', {}).get('offset', 0.)
                   for name, j in self.linkage.items()}
        transforms = link_transforms(self.urdf, targets)
        tcp_inverse = np.linalg.inv(transforms['sf_grasp_tcp'])
        result = []
        for name, collision, triangles in self.collisions:
            transform = tcp_inverse @ transforms[name] @ origin_transform(collision.find('origin'))
            result.append(triangles @ transform[:3, :3].T + transform[:3, 3])
        return np.concatenate(result)

    def _bore_envelopes(self, depth, thickness):
        if not all(math.isfinite(v) and v > 0 for v in (depth, thickness)) or depth <= thickness/2:
            raise ValueError('Invalid insertion depth or ring thickness')
        primary = self.linkage[self.settings['primary_joint']]
        lower, upper = primary['lower'], primary['upper']
        contracted = self.triangles(upper)
        slab = (-depth-thickness/2, -depth+thickness/2)
        contracted_radius = slab_radius(contracted, slab[0], max(0., float(contracted[:, :, 2].max())))
        expanded_radius = slab_radius(self.triangles(lower), *slab)
        return lower, upper, slab, contracted_radius, expanded_radius

    def solve_bore(self, depth, thickness, inner_radius):
        key=(float(depth),float(thickness),float(inner_radius))
        cache=getattr(self,'_bore_solutions',None)
        if cache is None:self._bore_solutions={};cache=self._bore_solutions
        if key not in cache:cache[key]=self._solve_bore_uncached(*key)
        return dict(cache[key])

    def _solve_bore_uncached(self, depth, thickness, inner_radius):
        """Solve selected fingers against material dimensions supplied by the task.

        The swept distal envelope must clear the bore by 1 mm at contraction;
        the expansion endpoint must reach its inner wall at the capture slab.
        """
        lower, upper, slab, contracted_radius, expanded_radius = self._bore_envelopes(depth, thickness)
        if (not math.isfinite(inner_radius) or inner_radius-contracted_radius < .001
                or inner_radius >= expanded_radius):
            raise ValueError('Bore is outside selected Tool insertion/expansion envelope')
        lo, hi = lower, upper
        for _ in range(40):
            mid = (lo+hi)/2
            if slab_radius(self.triangles(mid), *slab) > inner_radius:
                lo = mid
            else:
                hi = mid
        transforms = link_transforms(self.urdf, {})
        relative = np.linalg.inv(transforms[self.base_link]) @ transforms['sf_grasp_tcp']
        return dict(insertion_depth_m=depth, thickness_m=thickness, inner_radius_m=inner_radius,
                    contracted_swept_radius_m=contracted_radius, expanded_radius_m=expanded_radius,
                    release_position_m=upper, grasp_position_m=(lo+hi)/2,
                    tcp_z_m=float(relative[2, 3]))

    def fixture(self, depth, thickness):
        """Experiment-only ring generation; common strategies never infer material size."""
        _, _, _, contracted_radius, expanded_radius = self._bore_envelopes(depth, thickness)
        inner = (contracted_radius + expanded_radius)/2
        solution = self.solve_bore(depth, thickness, inner)
        return dict(solution, outer_radius_m=inner+.008,
                    contract_position_m=solution['release_position_m'])


def qualify_capture(relative_position, relative_rotation, measured_primary, fixture, tolerance):
    position, rotation = np.asarray(relative_position), np.asarray(relative_rotation)
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)) or not math.isfinite(measured_primary):
        raise ValueError('Nonfinite capture observation')
    if np.linalg.norm(position[:2]) > .0005 or np.max(np.abs(rotation-np.eye(3))) > .005:
        raise ValueError('Ring/TCP alignment rejected')
    if abs(position[2] + fixture['insertion_depth_m']) > .0005:
        raise ValueError('Ring insertion depth rejected')
    if abs(measured_primary-fixture['grasp_position_m']) > tolerance:
        raise ValueError('Measured expansion rejected')
