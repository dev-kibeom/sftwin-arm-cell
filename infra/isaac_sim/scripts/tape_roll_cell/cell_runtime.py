"""Configured Tape Cell runtime: inventory, measured completion and contact validity."""
import math
import time


def receiver_settled(position, velocity, angular_velocity, axis, expected):
    vectors = (position, velocity, angular_velocity, axis, expected)
    if any(len(v) != 3 or not all(math.isfinite(float(x)) for x in v) for v in vectors):
        return False
    norm = lambda v: math.sqrt(sum(float(x)**2 for x in v))
    length = norm(axis)
    return (norm([a-b for a,b in zip(position, expected)]) <= .01
            and norm(velocity) <= .02 and norm(angular_velocity) <= .2 and length > 0
            and math.acos(min(1., abs(axis[0])/length)) <= .05)


def update_identity_attributes(stage, paths, supply):
    from pxr import Sdf
    state = supply.state(valid=False)
    for i, path in enumerate(paths):
        prim = stage.GetPrimAtPath(path)
        values = dict(material_id=supply.material_id, supply_id=supply.supply_id,
            slot_id=f'slot_{i}', supply_epoch=supply.epoch,
            reservation_id=supply.reservation_id if supply._selected == i else '',
            occupied=state['occupied'][i])
        for name, value in values.items():
            kind = Sdf.ValueTypeNames.Bool if isinstance(value, bool) else Sdf.ValueTypeNames.String
            prim.CreateAttribute('arm_cell:'+name, kind, custom=True).Set(value)


class ConfiguredSupplyAdapter:
    def __init__(self, configuration, context, sidecar=None):
        import omni.timeline
        import omni.physx
        from pxr import UsdGeom, UsdPhysics, PhysxSchema
        from gripper_runtime.physical_contacts import PhysicalContactObserver
        self.configuration, self.context = configuration, context
        self.stage, self.paths, self.supply = context['stage'], context['roll_paths'], context['supply']
        self.timeline = omni.timeline.get_timeline_interface()
        self.tool_session = None
        self.failure = ''
        self._closed = False
        self._last_state_at = 0.
        self._last_feedback_generation = None
        self._last_feedback_at = 0.
        self._initial = [(p, UsdGeom.Xformable(self.stage.GetPrimAtPath(p)).GetLocalTransformation()) for p in self.paths]
        prefix = configuration['cell']['isaac']['robot_prim_path'].rstrip('/')+'/'
        self.robot_bodies = set()
        for prim in self.stage.Traverse():
            if str(prim.GetPath()).startswith(prefix) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                self.robot_bodies.add(str(prim.GetPath()))
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        self.observer = PhysicalContactObserver()
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_on_step_events(self._on_physics_step, True, 0)
        if sidecar is None:
            from tape_roll_cell.configured_supply_sidecar import ConfiguredSupplyClient
            try:
                sidecar = ConfiguredSupplyClient.launch()
            except Exception:
                self.observer.close()
                unsubscribe = getattr(self.physics_sub, 'unsubscribe', None)
                if callable(unsubscribe): unsubscribe()
                self.physics_sub = None
                raise
        self.sidecar = sidecar

    def set_tool_session(self, session):
        self.tool_session = session

    def valid(self):
        manager = getattr(self.tool_session, 'grasp_manager', None)
        generation = getattr(self.tool_session, '_feedback_generation', 0)
        now = time.monotonic()
        if generation > 0 and generation != self._last_feedback_generation:
            self._last_feedback_generation, self._last_feedback_at = generation, now
        return (generation > 0 and now-self._last_feedback_at <= .5 and not self._closed and not self.failure and self.timeline.is_playing()
                and bool(getattr(manager, 'ready', False)) and self.sidecar.available)

    def _release_observed(self, slot):
        from gripper_runtime.grasp_attachment import HoldingState
        manager = getattr(self.tool_session, 'grasp_manager', None)
        if manager is None:
            return False
        receipt = getattr(manager, 'release_receipt', None)
        reference = receipt() if callable(receipt) else None
        if not isinstance(reference, dict):
            return False
        observation = manager.holding_observation()
        return (observation.state is HoldingState.RELEASED and observation.fresh()
                and bool(reference.get('execution_id'))
                and reference.get('supply_id') == self.supply.supply_id
                and reference.get('material_id') == self.supply.material_id
                and reference.get('slot_id') == f'slot_{slot}'
                and reference.get('supply_epoch') == self.supply.epoch
                and reference.get('reservation_id') == self.supply.reservation_id)

    def _completion_observed(self, slot):
        from pxr import UsdGeom, UsdPhysics
        if not self._release_observed(slot):
            return False
        prim = self.stage.GetPrimAtPath(self.paths[slot])
        if not prim:
            return False
        # Read solver state, rather than USD velocity defaults which need not
        # be written back by every Isaac physics configuration.
        from omni.isaac.dynamic_control import _dynamic_control as api
        dc = api.acquire_dynamic_control_interface()
        handle = dc.get_rigid_body(self.paths[slot])
        if handle == api.INVALID_HANDLE:
            return False
        pose = dc.get_rigid_body_pose(handle)
        velocity = dc.get_rigid_body_linear_velocity(handle)
        angular = dc.get_rigid_body_angular_velocity(handle)
        x,y,z,w = pose.r.x,pose.r.y,pose.r.z,pose.r.w
        axis = (2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y))
        expected = self.configuration['cell']['tape_roll']['slots'][slot]['receiver_center_m']
        return receiver_settled((pose.p.x,pose.p.y,pose.p.z),
            (velocity.x,velocity.y,velocity.z), (angular.x,angular.y,angular.z), axis, expected)

    def _on_physics_step(self, dt):
        self.observer.advance()
        if self.observer.snapshot is None:
            self.failure = 'ambiguous_contact_step'
            return
        if not self.timeline.is_playing():
            return
        manager = getattr(self.tool_session, 'grasp_manager', None)
        binding = getattr(manager, 'binding', None)
        fingers = set(getattr(binding, 'finger_bodies', ()))
        selected = self.paths[self.supply._selected] if self.supply._selected is not None else None
        root = self.context['root_path']+'/'
        for contact in self.observer.snapshot:
            if contact['lost']:
                continue
            a, b = contact['actor0'], contact['actor1']
            robot = a if a in self.robot_bodies else b if b in self.robot_bodies else None
            other = b if robot == a else a
            if robot is None:
                continue
            if other in self.paths:
                if other != selected or robot not in fingers:
                    if not self.failure:print(f'Tape Cell unexpected contact: {a} / {b}',flush=True)
                    self.failure = 'unexpected_robot_material_contact'
            elif other.startswith(root):
                if not self.failure:print(f'Tape Cell fixture contact: {a} / {b}',flush=True)
                self.failure = 'robot_fixture_collision'

    def spin_once(self):
        if self._closed:
            return
        for packet in self.sidecar.poll_messages():
            if packet.get('op') == 'mission_status':
                self.last_mission_status = packet.get('text', '')
            if packet.get('op') != 'request':
                continue
            try:
                if time.monotonic() > float(packet.get('deadline_monotonic', 0.)):
                    result = self.supply._response(False, 'expired_request')
                    self.sidecar.send(dict(op='response', correlation_id=packet.get('correlation_id'), response=result))
                    continue
                result = self.supply.handle(packet['request'], valid=self.valid(),
                    completion_observed=self._completion_observed)
                update_identity_attributes(self.stage, self.paths, self.supply)
            except Exception as error:
                self.failure = f'configured_supply_error: {error}'
                result = self.supply._response(False, self.failure)
            self.sidecar.send(dict(op='state', state=self.supply.state(valid=self.valid(), detail=self.failure), scene=self.scene_packet()))
            self.sidecar.send(dict(op='response', correlation_id=packet.get('correlation_id'), response=result))
        now = time.monotonic()
        if now-self._last_state_at >= .1:
            self._last_state_at = now
            self.sidecar.send(dict(op='state', state=self.supply.state(valid=self.valid(), detail=self.failure), scene=self.scene_packet()))
        self.sidecar.flush_outgoing()

    def scene_packet(self):
        from pxr import UsdGeom, UsdPhysics
        from dataclasses import asdict
        cache = UsdGeom.XformCache()
        rolls = []
        for i, path in enumerate(self.paths):
            matrix = cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(path))
            quaternion = matrix.ExtractRotationQuat()
            rolls.append(dict(id=f'{self.supply.supply_id}_slot_{i}', slot_id=f'slot_{i}',
                position=list(matrix.ExtractTranslation()),
                orientation_xyzw=[*quaternion.GetImaginary(), quaternion.GetReal()]))
        manager = getattr(self.tool_session, 'grasp_manager', None)
        observation = manager.holding_observation() if manager is not None else None
        state = self.supply.state(valid=self.valid())
        # Static box dimensions are read from authored scale ops, not inferred
        # from a rotation matrix whose scale extraction could vary by SDK.
        static = []
        for prim in self.stage.Traverse():
            if (str(prim.GetPath()).startswith(self.context['root_path']+'/')
                    and prim.IsA(UsdGeom.Cube) and prim.HasAPI(UsdPhysics.CollisionAPI)):
                transform = cache.GetLocalToWorldTransform(prim)
                scale = prim.GetAttribute('xformOp:scale').Get()
                static.append(dict(id=str(prim.GetPath()), position=list(transform.ExtractTranslation()),
                                   dimensions=list(scale)))
        return dict(frame_id=self.configuration['robot']['motion'].get('world_frame', 'world'), rolls=rolls,
            material=asdict(self.context['roll']), selected_slot_id=state['selected_slot_id'],
            reservation_id=state['reservation_id'], supply_epoch=self.supply.epoch,
            holding_state=observation.state.value if observation is not None and observation.fresh() else 'unknown',
            accepted_relation=getattr(manager, 'accepted_relation', None),
            accepted_execution_id=getattr(manager, 'accepted_execution_id', None),
            finger_links=list(self.configuration['tool']['runtime']['finger_tip_links']), static_boxes=static)

    def reset_generated_materials(self, attachment_reset):
        from pxr import Gf, UsdGeom, UsdPhysics
        if not self.timeline.is_stopped() or not attachment_reset():
            return False
        for path, matrix in self._initial:
            prim = self.stage.GetPrimAtPath(path)
            if not prim:
                self.failure = 'material_missing_during_reset'
                return False
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(matrix)
            body = UsdPhysics.RigidBodyAPI(prim)
            body.CreateVelocityAttr(Gf.Vec3f(0.))
            body.CreateAngularVelocityAttr(Gf.Vec3f(0.))
        self.supply.reset()
        self.failure = ''
        self.observer.clear()
        update_identity_attributes(self.stage, self.paths, self.supply)
        self.sidecar.send(dict(op='state', state=self.supply.state(valid=False, detail='timeline_stopped')))
        self.sidecar.flush_outgoing()
        return True

    def set_live_test_session_id(self, session_id, root=None):
        self.live_test_session_id = session_id

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.physics_sub is not None:
            unsubscribe = getattr(self.physics_sub, 'unsubscribe', None)
            if callable(unsubscribe): unsubscribe()
            self.physics_sub = None
        self.observer.close()
        self.sidecar.close()


def create_adapter(*, configuration, context):
    return ConfiguredSupplyAdapter(configuration, context)
