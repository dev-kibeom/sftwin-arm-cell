"""Cell-owned material identities for read-only simulation diagnostics."""
_source = None


def register_material_source(owner, materials):
    global _source
    _source = (owner, materials)


def clear_material_source(owner):
    global _source
    if _source is not None and _source[0] is owner:
        _source = None


def material_observation_targets():
    return list(_source[1]()) if _source is not None else []


class DynamicBodyObserver:
    """Read current PhysX bodies, including objects spawned after Play."""
    def __init__(self):
        from omni.isaac.dynamic_control import _dynamic_control as api
        self.api = api
        self.dc = api.acquire_dynamic_control_interface()
        self._tcp_binding = None
        self._handles = {}

    def prepare_tcp(self, path):
        from pxr import Gf, UsdGeom, UsdPhysics
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        tcp = stage.GetPrimAtPath(path)
        body = tcp
        while body.IsValid() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        if not body.IsValid():
            return
        cache = UsdGeom.XformCache()
        local = cache.GetLocalToWorldTransform(tcp) * cache.GetLocalToWorldTransform(body).GetInverse()
        rotation = local.ExtractRotationQuat()
        self._tcp_binding = (path, str(body.GetPath()),
            [*local.ExtractTranslation(), *rotation.GetImaginary(), rotation.GetReal()])

    def tcp(self, path):
        if not self._tcp_binding or self._tcp_binding[0] != path:
            return self.body(path)
        _, body_path, local_pose = self._tcp_binding
        return tcp_state_from_body(self.body(body_path), local_pose, path)

    def body(self, path):
        try:
            return self._read_body(path)
        except Exception as error:
            self._handles.pop(path, None)
            return dict(status='unavailable', prim_path=path,
                        reason=f'{type(error).__name__}: {error}')

    def _read_body(self, path):
        handle = self._handles.get(path)
        if handle is None:
            handle = self.dc.get_rigid_body(path)
        if handle == self.api.INVALID_HANDLE:
            return dict(status='unavailable', reason='rigid_body_unavailable', prim_path=path)
        if len(self._handles) >= 64:
            self._handles.clear()
        self._handles[path] = handle
        pose = self.dc.get_rigid_body_pose(handle)
        linear = self.dc.get_rigid_body_linear_velocity(handle)
        angular = self.dc.get_rigid_body_angular_velocity(handle)
        vector = lambda value: [float(value.x), float(value.y), float(value.z)]
        values = dict(position_m=vector(pose.p),
            orientation_xyzw=[float(pose.r.x), float(pose.r.y), float(pose.r.z), float(pose.r.w)],
            linear_velocity_m_s=vector(linear), angular_velocity_rad_s=vector(angular))
        import math
        if not all(math.isfinite(v) for field in values.values() for v in field):
            return dict(status='unavailable', reason='nonfinite_rigid_body_state', prim_path=path)
        return dict(status='observed', source='physx_dynamic_control', prim_path=path, **values)

    def snapshot(self, tcp_path, materials, max_materials=16):
        targets = sorted(materials, key=lambda item: not item.get('selected', False))
        return dict(source='physx_dynamic_control', frame_id='world',
            tcp=self.tcp(tcp_path) if tcp_path else dict(status='unavailable', reason='tcp_rigid_body_unavailable'),
            materials=[dict(identity=item, state=self.body(item['prim_path']))
                       for item in targets[:max_materials]],
            coverage=dict(material_count=len(targets), retained_material_count=min(len(targets), max_materials),
                          omitted_material_count=max(0, len(targets)-max_materials)))


def tcp_state_from_body(state, local_pose, tcp_path):
    """Apply the fixed TCP offset to one measured body state, including twist."""
    if state['status'] != 'observed':
        return state
    def cross(a, b):
        return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    q, local_q = state['orientation_xyzw'], local_pose[3:]
    t = [2*v for v in cross(q[:3], local_pose[:3])]
    rotated = cross(q[:3], t)
    offset = [local_pose[i] + q[3]*t[i] + rotated[i] for i in range(3)]
    vector = cross(q[:3], local_q[:3])
    orientation = [q[3]*local_q[i] + local_q[3]*q[i] + vector[i] for i in range(3)]
    orientation.append(q[3]*local_q[3] - sum(q[i]*local_q[i] for i in range(3)))
    tangential = cross(state['angular_velocity_rad_s'], offset)
    return dict(state, prim_path=tcp_path, rigid_body_prim_path=state['prim_path'],
        observation_point='tcp_from_physx_body_and_fixed_transform',
        position_m=[state['position_m'][i]+offset[i] for i in range(3)],
        orientation_xyzw=orientation,
        linear_velocity_m_s=[state['linear_velocity_m_s'][i]+tangential[i] for i in range(3)])
