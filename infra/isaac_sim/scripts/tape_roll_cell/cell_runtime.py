"""Device-driven Tape sequences with independent collision Ground Truth evidence."""
import logging
import math
import time


LOGGER = logging.getLogger(__name__)

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
        self.observer = None
        try:
            for prim in self.stage.Traverse():
                if str(prim.GetPath()).startswith(prefix) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    self.robot_bodies.add(str(prim.GetPath()))
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            self.observer = PhysicalContactObserver()
        except Exception:
            # Missing GT observation is unavailable evidence, never a Cell fault.
            self.observer = None
        self._physics_step_sequence = 0
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_on_step_events(self._on_physics_step, True, 0)
        self.observation_sub = None
        try:
            self.observation_sub = omni.physx.get_physx_interface().subscribe_physics_on_step_events(
                self._on_physics_observation, False, 0)
        except Exception:
            self._contact_observer_call('close')
            self.observer = None
        if sidecar is None:
            from tape_roll_cell.configured_supply_sidecar import ConfiguredSupplyClient
            try:
                sidecar = ConfiguredSupplyClient.launch()
            except Exception:
                self._contact_observer_call('close')
                unsubscribe = getattr(self.physics_sub, 'unsubscribe', None)
                if callable(unsubscribe): unsubscribe()
                self.physics_sub = None
                raise
        self.sidecar = sidecar
        from tape_roll_cell.conveyor import Conveyor
        from isaac_runtime.cell_supply import register_supply_port
        self.conveyor=Conveyor(self.stage,configuration,context['roll'])
        self._cart_number=0;self._pending_cycle=None;self._mission_available=False;self._mission_available_at=0.
        self._cycle_slot=None;self.last_mission_status=''
        self._cycle_evidence=None;self._cycle_writer=None
        register_supply_port(self,dict(materials=[dict(id=self.supply.material_id,label='Tape Roll 17 mm')],
            count_label=f'Full cartridges ({self.supply._count} rolls each)',default_count=1),
            self.request_supply,self.supply_status)
        from tape_roll_cell.teaching_runtime import TeachingSession
        self.teaching = TeachingSession(self)

    def request_supply(self,material_id,count):
        if not self.timeline.is_playing():return dict(accepted=False,detail='Press Play before requesting Supply.')
        if material_id!=self.supply.material_id:return dict(accepted=False,detail='Unsupported material')
        response=self.conveyor.enqueue_cartridges(count)
        if response['accepted']:
            self.last_mission_status=''
        return response

    def supply_status(self):
        motion=self.configuration['cell'].get('motion_preparation',dict(ready=True,detail=''))
        detail=(f'Scene ready · Motion preparation unavailable: {motion["detail"]}' if not motion['ready'] else
                self.conveyor.detail)
        return dict(available=not self._closed,state=self.conveyor.state,queued=self.conveyor.queued,
            accepted_cartridges=self.conveyor.accepted,
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
        if (not getattr(getattr(self, 'teaching', None), 'manual_mode', False) and
                self.valid() and ready['ready'] and self._pending_cycle is None and
                not self.supply.reservation_id and self._mission_available and
                time.monotonic()-self._mission_available_at<.5):
            import uuid
            self._pending_cycle=uuid.uuid4().hex
            self._cycle_slot=self.conveyor.current_ordinal
            if self._cycle_evidence is not None:
                self._contact_observer_call('clear')
                self._cycle_evidence.start(self._pending_cycle, supply_epoch=self.supply.epoch,
                    slot_id=f'slot_{self._cycle_slot}')
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

    def _contact_observer_call(self, operation):
        try:
            if self.observer is not None:
                getattr(self.observer, operation)()
        except Exception:
            self.observer = None

    def _on_physics_step(self, dt):
        if not hasattr(self,'conveyor'):return
        if not self.timeline.is_playing():
            self._contact_observer_call('clear')
            return
        self.conveyor.on_physics_step(dt)
        # Count executed steps on the device-driving callback, independently of
        # the optional contact callback. This evidence never controls execution.
        self._physics_step_sequence += 1
        try:
            if self._cycle_evidence is not None:
                self._cycle_evidence.physics_step(self._physics_step_sequence)
        except Exception:
            self._cycle_evidence = None

    def _on_physics_observation(self, dt):
        if not self.timeline.is_playing():
            self._contact_observer_call('clear')
            return
        self._observe_cycle_contacts()

    def _observe_cycle_contacts(self, physics_step=True):
        # PhysX identities and contact reports belong only to verification.
        # Even an unavailable observer cannot invalidate the device-driven Cell.
        self._contact_observer_call('advance')
        if self._cycle_evidence is None or self._cycle_evidence.active is None:
            return
        try:
            snapshot = self.observer.snapshot if self.observer is not None else None
            if snapshot is None:
                self._cycle_evidence.observe(None)
                return
            manager = getattr(self.tool_session, 'grasp_manager', None)
            fingers = set(getattr(getattr(manager, 'binding', None), 'finger_bodies', ()))
            slot = getattr(self, '_cycle_slot', self.supply._selected)
            selected = self.paths[slot] if slot is not None and 0 <= slot < len(self.paths) else None
            collisions = []
            for contact in snapshot:
                if contact['lost'] or not physical_contact_observed(contact):
                    continue
                a, b = contact['actor0'], contact['actor1']
                robot = a if a in self.robot_bodies else b if b in self.robot_bodies else None
                other = b if robot == a else a
                if robot is None:
                    continue
                if other == selected and robot in fingers:
                    continue
                if (other == self.context['root_path']+'/RobotPedestal' and
                    robot == self.configuration['cell']['isaac']['robot_prim_path']+'/'+
                        self.configuration['model_configuration']['isaac']['articulation_base']):
                    continue
                collisions.append(contact)
            self._cycle_evidence.observe(collisions,
                step_id=getattr(self, '_physics_step_sequence', None), physics_step=physics_step)
        except Exception:
            self._cycle_evidence.observe(None)

    def spin_once(self):
        if self._closed:
            return
        self._advance_supply()
        self._poll_sidecar()

    def spin_teaching_once(self):
        """Process pendant responses while stopped without advancing Supply."""
        if not self._closed:
            self._poll_sidecar()

    def _poll_sidecar(self):
        for packet in self.sidecar.poll_messages():
            if packet.get('op', '').startswith('teaching_'):
                self.teaching.consume(packet)
                continue
            if packet.get('op')=='mission_available':
                self._mission_available=packet.get('ready') is True;self._mission_available_at=time.monotonic()
            if packet.get('op')=='mission_result' and packet.get('request_id')==self._pending_cycle:
                self._pending_cycle=None
                succeeded=packet.get('success') is True
                if self._cycle_evidence is not None:
                    self._observe_cycle_contacts(physics_step=False)
                    self._cycle_evidence.finish(succeeded)
                if succeeded:
                    self.conveyor.release_to_outfeed(self.paths[self._cycle_slot])
                elif not self.failure:
                    self.failure='Teaching cycle failed: '+str(packet.get('detail','Unknown failure'))
            if packet.get('op') == 'mission_status':
                self.last_mission_status = packet.get('text', '')
            if packet.get('op') != 'request':
                continue
            try:
                if time.monotonic() > float(packet.get('deadline_monotonic', 0.)):
                    result = self.supply._response(False, 'expired_request')
                    self.sidecar.send(dict(op='response', correlation_id=packet.get('correlation_id'), response=result))
                    continue
                request=packet['request']
                release_valid=(request.get('command') != 1 or self._release_observed(self.supply._selected))
                result = self.supply.handle(request, valid=self.valid() and release_valid)
                update_identity_attributes(self.stage, self.paths, self.supply)
            except Exception as error:
                self.failure = f'configured_supply_error: {error}'
                result = self.supply._response(False, self.failure)
            self.sidecar.send(dict(op='state', state=self.supply.state(valid=self.valid(), detail=self.failure)))
            self.sidecar.send(dict(op='response', correlation_id=packet.get('correlation_id'), response=result))
        now = time.monotonic()
        if now-self._last_state_at >= .1:
            self._last_state_at = now
            self.sidecar.send(dict(op='state', state=self.supply.state(valid=self.valid(), detail=self.failure)))
        self.sidecar.flush_outgoing()

        self.teaching.tick()

    def reset_generated_materials(self, attachment_reset):
        if not self.timeline.is_stopped() or not attachment_reset():
            return False
        if getattr(self, 'teaching', None) is not None:
            self.teaching.on_stop()
        self.conveyor.resetstopped()
        self.paths=[];self._pending_cycle=None;self._mission_available=False
        self.last_mission_status=''
        if self._cycle_evidence is not None:
            self._cycle_evidence.finish(False)
        self.sidecar.send(dict(op='cancel_cycle'))
        self.supply.reset();self.supply.unload()
        self.failure = ''
        self._contact_observer_call('clear')
        update_identity_attributes(self.stage, self.paths, self.supply)
        self.sidecar.send(dict(op='state', state=self.supply.state(valid=False, detail='timeline_stopped')))
        self.sidecar.flush_outgoing()
        return True

    def set_live_test_session_id(self, session_id, root=None):
        if self._cycle_evidence is not None:
            self._cycle_evidence.finish(False)
        if self._cycle_writer is not None:
            try:
                self._cycle_writer.close()
            except Exception:
                pass
        self._cycle_evidence=self._cycle_writer=None
        self.live_test_session_id = session_id
        if session_id is not None and root is not None:
            from pathlib import Path
            from gripper_runtime.capture_diagnostics import BoundedCaptureDiagnosticWriter
            from tape_roll_cell.cycle_evidence import CycleEvidence
            try:
                self._cycle_writer=BoundedCaptureDiagnosticWriter(Path(root)/'simulation'/'tape_cycles')
                self._cycle_evidence=CycleEvidence(self._cycle_writer.submit, session_id)
            except Exception:
                self._cycle_writer=self._cycle_evidence=None

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.physics_sub is not None:
            unsubscribe = getattr(self.physics_sub, 'unsubscribe', None)
            if callable(unsubscribe): unsubscribe()
            self.physics_sub = None
        if self.observation_sub is not None:
            try:
                unsubscribe = getattr(self.observation_sub, 'unsubscribe', None)
                if callable(unsubscribe): unsubscribe()
            except Exception:
                pass
            self.observation_sub = None
        self.set_live_test_session_id(None)
        self._contact_observer_call('close')
        self.teaching.close()
        from isaac_runtime.cell_supply import clear_supply_port
        clear_supply_port(self)
        self.conveyor.close()
        self.sidecar.close()


def create_adapter(*, configuration, context):
    return ConfiguredSupplyAdapter(configuration, context)
