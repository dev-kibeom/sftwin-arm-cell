"""Configured Tape Cell runtime: inventory, measured completion and contact validity."""
import math
import time


def physical_contact_observed(report):
    """PhysX also reports separated shapes inside their contact offsets.

    Exclude only proven positive-gap, zero-impulse reports. Missing or
    nonfinite evidence remains conservative, as do touching/loaded points.
    """
    points = report.get('points')
    if not points:
        return True
    for point in points:
        try:
            separation = float(point['separation_m'])
            impulse = tuple(float(value) for value in point['impulse'])
        except (KeyError, TypeError, ValueError):
            return True
        if (not math.isfinite(separation) or separation <= 0.
                or len(impulse) != 3 or any(not math.isfinite(value) or value != 0. for value in impulse)):
            return True
    return False


def receiver_settled(position, velocity, angular_velocity, axis, expected):
    vectors = (position, velocity, angular_velocity, axis, expected)
    if any(len(v) != 3 or not all(math.isfinite(float(x)) for x in v) for v in vectors):
        return False
    norm = lambda v: math.sqrt(sum(float(x)**2 for x in v))
    length = norm(axis)
    return (norm([a-b for a,b in zip(position, expected)]) <= .01
            and norm(velocity) <= .02 and norm(angular_velocity) <= .2 and length > 0
            and math.acos(min(1., abs(axis[2])/length)) <= .05)


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
        from pxr import UsdPhysics, PhysxSchema
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
        from tape_roll_cell.conveyor import Conveyor
        from isaac_runtime.cell_supply import register_supply_port
        self.conveyor=Conveyor(self.stage,configuration,context['roll'])
        self._cart_number=0;self._pending_cycle=None;self._mission_available=False;self._mission_available_at=0.
        self._cycle_start_count=0;self.last_mission_status=''
        self._completed_total=0
        register_supply_port(self,dict(materials=[dict(id=self.supply.material_id,label='Tape Roll 17 mm')],
            count_label=f'가득 찬 카트리지 수 ({self.supply._count}롤/개)',default_count=1),
            self.request_supply,self.supply_status)

    def request_supply(self,material_id,count):
        if not self.timeline.is_playing():return dict(accepted=False,detail='Timeline Play 후 Supply를 요청하세요.')
        if material_id!=self.supply.material_id:return dict(accepted=False,detail='지원하지 않는 자재')
        response=self.conveyor.enqueue_cartridges(count)
        if response['accepted']:
            self._completed_total=0;self.last_mission_status=''
        return response

    def supply_status(self):
        total=self.conveyor.accepted*self.supply._count
        motion=self.configuration['cell'].get('motion_preparation',dict(ready=True,detail=''))
        detail=(f'장면 준비됨 · 동작 경로 준비 불가: {motion["detail"]}' if not motion['ready'] else
                f'{self.conveyor.detail} · 완료 {self._completed_total}/{total}롤')
        return dict(available=not self._closed,state=self.conveyor.state,queued=self.conveyor.queued,
            completed_cartridges=self.conveyor.finished,accepted_cartridges=self.conveyor.accepted,
            completed_rolls=self._completed_total,remaining_rolls=total-self._completed_total,
            motion_ready=motion['ready'],detail=self.failure or detail,
            mission=self.last_mission_status)

    def _advance_supply(self):
        self.conveyor.update()
        paths=self.conveyor.active_roll_paths
        if paths and self.conveyor.number!=self._cart_number:
            self.paths=list(paths);self._cart_number=self.conveyor.number
            self.supply.reset()
            update_identity_attributes(self.stage,self.paths,self.supply)
        elif not paths:self.paths=[]
        ready=self.configuration['cell'].get('motion_preparation',{'ready':True})
        if (self.valid() and ready['ready'] and self._pending_cycle is None and
                not self.supply.reservation_id and self._mission_available and
                time.monotonic()-self._mission_available_at<.5):
            import uuid
            self._pending_cycle=uuid.uuid4().hex
            self._cycle_start_count=self.supply.snapshot()['completed']
            self.sidecar.send(dict(op='start_cycle',request_id=self._pending_cycle,
                target_id=self.supply.material_id,quantity=1))

    def set_tool_session(self, session):
        self.tool_session = session

    def valid(self):
        manager = getattr(self.tool_session, 'grasp_manager', None)
        generation = getattr(self.tool_session, '_feedback_generation', 0)
        now = time.monotonic()
        if generation > 0 and generation != self._last_feedback_generation:
            self._last_feedback_generation, self._last_feedback_at = generation, now
        return (generation > 0 and now-self._last_feedback_at <= .5 and not self._closed and not self.failure and self.timeline.is_playing()
                and bool(getattr(manager, 'ready', False)) and self.sidecar.available and self.conveyor.dock_ready)

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
        if not hasattr(self,'conveyor'):return
        if not self.timeline.is_playing():
            self.observer.clear()
            return
        self.conveyor.on_physics_step(dt)
        self.observer.advance()
        if self.observer.snapshot is None:
            if not self.failure:
                print('Tape Cell contact observation crossed physics steps', flush=True)
            self.failure = 'ambiguous_contact_step'
            return
        manager = getattr(self.tool_session, 'grasp_manager', None)
        binding = getattr(manager, 'binding', None)
        fingers = set(getattr(binding, 'finger_bodies', ()))
        selected = self.paths[self.supply._selected] if self.supply._selected is not None else None
        root = self.context['root_path']+'/'
        for contact in self.observer.snapshot:
            if contact['lost'] or not physical_contact_observed(contact):
                continue
            a, b = contact['actor0'], contact['actor1']
            robot = a if a in self.robot_bodies else b if b in self.robot_bodies else None
            other = b if robot == a else a
            if robot is None:
                continue
            if other in self.paths:
                if other != selected or robot not in fingers:
                    if not self.failure:print(f'Tape Cell unexpected contact: {a} / {b} selected={selected} fingers={sorted(fingers)} points={contact["points"][:3]}',flush=True)
                    self.failure = 'unexpected_robot_material_contact'
            elif (other == self.context['root_path']+'/RobotPedestal'
                    and robot == self.configuration['cell']['isaac']['robot_prim_path']+'/'+
                        self.configuration['model_configuration']['isaac']['articulation_base']):
                continue
            elif other.startswith(root):
                if not self.failure:print(f'Tape Cell fixture contact: {a} / {b}',flush=True)
                self.failure = 'robot_fixture_collision'

    def spin_once(self):
        if self._closed:
            return
        self._advance_supply()
        for packet in self.sidecar.poll_messages():
            if packet.get('op')=='mission_available':
                self._mission_available=packet.get('ready') is True;self._mission_available_at=time.monotonic()
            if packet.get('op')=='mission_result' and packet.get('request_id')==self._pending_cycle:
                self._pending_cycle=None
                count=self.supply.snapshot()['completed']
                if packet.get('success') is True and count==self._cycle_start_count+1:
                    self.conveyor.release_to_outfeed(self.paths[count-1])
                elif not self.failure:
                    self.failure='PnP 실패: '+str(packet.get('detail','완료 수량 불일치'))
            if packet.get('op') == 'mission_status':
                self.last_mission_status = packet.get('text', '')
            if packet.get('op') != 'request':
                continue
            try:
                if time.monotonic() > float(packet.get('deadline_monotonic', 0.)):
                    result = self.supply._response(False, 'expired_request')
                    self.sidecar.send(dict(op='response', correlation_id=packet.get('correlation_id'), response=result))
                    continue
                before=self.supply.snapshot()['completed']
                result = self.supply.handle(packet['request'], valid=self.valid(),
                    completion_observed=self._completion_observed)
                self._completed_total+=self.supply.snapshot()['completed']-before
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
        from pxr import Gf, UsdGeom, UsdPhysics
        from dataclasses import asdict
        cache = UsdGeom.XformCache()
        rolls = []
        for number,paths in self.conveyor.history:
            for i,path in enumerate(paths):
                prim=self.stage.GetPrimAtPath(path)
                if not prim:continue
                matrix=cache.GetLocalToWorldTransform(prim);quaternion=matrix.ExtractRotationQuat()
                active=number==self._cart_number and bool(self.conveyor.active_roll_paths)
                slot_id=f'slot_{i}' if active else f'cartridge_{number}_slot_{i}'
                identity=f'{self.supply.supply_id}_{slot_id}'
                rolls.append(dict(id=identity,slot_id=slot_id,
                    support_prefix=f'{self.conveyor.root}/Carrier{number:04d}',position=list(matrix.ExtractTranslation()),
                    orientation_xyzw=[*quaternion.GetImaginary(),quaternion.GetReal()]))
        manager = getattr(self.tool_session, 'grasp_manager', None)
        observation = manager.holding_observation() if manager is not None else None
        state = self.supply.state(valid=self.valid())
        # Fixed fixtures retain their Cell-owned canonical geometry, also used
        # by the startup scene loader. USD float scales must not overwrite it
        # during the loader's readback. Only moving pallet boxes are measured.
        fixtures = {item['id']: item for item in self.configuration['cell']['tape_roll']['static_boxes']}
        static = []
        for prim in self.stage.Traverse():
            if (str(prim.GetPath()).startswith(self.context['root_path']+'/')
                    and prim.IsA(UsdGeom.Cube) and prim.HasAPI(UsdPhysics.CollisionAPI)):
                fixed = fixtures.get(str(prim.GetPath()))
                if fixed is not None:
                    static.append(dict(id=fixed['id'],position=list(fixed['pose']['position_m']),
                        orientation_xyzw=list(fixed['pose']['quaternion_xyzw']),
                        dimensions=list(fixed['geometry']['dimensions_m'])))
                    continue
                transform = cache.GetLocalToWorldTransform(prim)
                scale = prim.GetAttribute('xformOp:scale').Get()
                quaternion = Gf.Transform(transform).GetRotation().GetQuat()
                static.append(dict(id=str(prim.GetPath()), position=list(transform.ExtractTranslation()),
                    orientation_xyzw=[*quaternion.GetImaginary(),quaternion.GetReal()],dimensions=list(scale)))
        return dict(frame_id=self.configuration['robot']['motion'].get('world_frame', 'world'), rolls=rolls,
            material=asdict(self.context['roll']), selected_slot_id=state['selected_slot_id'],
            reservation_id=state['reservation_id'], supply_epoch=self.supply.epoch,
            holding_state=observation.state.value if observation is not None and observation.fresh() else 'unknown',
            accepted_relation=getattr(manager, 'accepted_relation', None),
            accepted_execution_id=getattr(manager, 'accepted_execution_id', None),
            finger_links=list(self.configuration['tool']['runtime']['finger_tip_links']), static_boxes=static,
            mount_links=self.configuration['model_configuration']['mount_collision_links'],
            mount_object=self.context['root_path']+'/RobotPedestal')

    def reset_generated_materials(self, attachment_reset):
        if not self.timeline.is_stopped() or not attachment_reset():
            return False
        self.conveyor.resetstopped()
        self.paths=[];self._pending_cycle=None;self._mission_available=False
        self._completed_total=0;self.last_mission_status=''
        self.sidecar.send(dict(op='cancel_cycle'))
        self.supply.reset();self.supply.unload()
        self.failure = ''
        self.observer.clear()
        update_identity_attributes(self.stage, self.paths, self.supply)
        self.sidecar.send(dict(op='state', state=self.supply.state(valid=False, detail='timeline_stopped'),scene=self.scene_packet()))
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
        from isaac_runtime.cell_supply import clear_supply_port
        clear_supply_port(self)
        self.conveyor.close()
        self.sidecar.close()


def create_adapter(*, configuration, context):
    return ConfiguredSupplyAdapter(configuration, context)
