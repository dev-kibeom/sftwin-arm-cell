"""Isaac owns the physical receipt and advances the equipment only on simulation steps."""
from dataclasses import asdict
from pathlib import Path
import logging
import time

from isaac_runtime.cell_supply import clear_supply_port
from tape_roll_cell.configured_supply_sidecar import ConfiguredSupplyClient
from .equipment_io import Commands, Sensors
from .supply import ConveyorFeeder, material_instance_id

MAX_AGE = .9


class ConveyorRuntime:
    def __init__(self, configuration, equipment, sidecar):
        self.equipment, self.sidecar = equipment, sidecar
        cell = configuration['cell']
        layout = cell['conveyor']
        self.feeder = ConveyorFeeder(cell['configured_supply']['material_id'], last_generated=0,
                                     material_type_ids=layout['material_type_ids'])
        self.packet = None
        self.reservation = None
        self.retry_at = 0.
        self.startup_deadline = None
        self.synchronized = False
        self.stopped = False
        self.failure = ''
        self.last_sample = 0.
        self.sidecar_config = dict(
            op='configure', connection=layout['connection'], permit_topic=layout['equipment_permit_topic'],
            supply=dict(cell['configured_supply'], slot_id=layout['pick_slot_id'],
                        stream_id=equipment.stream_id, material_type_ids=layout['material_type_ids']),
            pick_configured=bool(layout['teaching']['cycle']),
            receipt_file=layout.get('receipt_file'),
            history_db_path=layout.get('history_db_path', '~/.local/state/arm_cell/conveyor-pnp-poc/production.sqlite'))
        self.sidecar.send(self.sidecar_config)
        self.feeder.bind_ui(set_automatic=self.set_automatic)

    def reconnect(self, *, wait_ready):
        self.packet = None
        self.sidecar.close()
        self.sidecar = ConfiguredSupplyClient.launch(Path(__file__).with_name('sidecar.py'), wait_ready=wait_ready)
        self.sidecar.send(dict(self.sidecar_config, resume_stream=True))
        self.startup_deadline = None if wait_ready else time.monotonic() + 15.
        if not self.sidecar.available:
            raise RuntimeError('Equipment sidecar disconnected during configuration')

    def set_automatic(self, enabled):
        if enabled and self.stopped:
            return dict(accepted=False, detail='Timeline stopped; reconcile the equipment stream.')
        if enabled and not self.sidecar.available:
            self.equipment.drive(0.)
            self.feeder.set_automatic(False)
            try:
                self.reconnect(wait_ready=True)
            except (OSError, RuntimeError) as error:
                return dict(accepted=False, detail=f'Equipment sidecar unavailable: {error}')
        return self.feeder.set_automatic(enabled)

    def set_tool_session(self, session):
        session.tool.capture.resolve_reserved_body = self.reserved_body

    def reserved_body(self, reference):
        if self.stopped:
            return None
        reservation = self.reservation
        if not reservation or any(reference.get(key) != reservation.get(key) for key in (
                'material_id', 'supply_id', 'slot_id', 'supply_epoch', 'reservation_id',
                'material_type_id', 'material_instance_id')):
            return None
        return self.equipment.material_path(reference['material_type_id'], reference['material_instance_id'])

    def tick(self, dt):
        now = time.monotonic()
        for packet in self.sidecar.poll_messages():
            if packet.get('op') == 'ready':
                self.startup_deadline = None
                continue
            self.packet = packet if packet.get('op') == 'command' else None
            if self.packet and not packet.get('observation_pending') and now - packet['at'] < MAX_AGE:
                self.reservation = packet.get('reservation')
        if self.startup_deadline is not None and now >= self.startup_deadline:
            self.sidecar.close()
            self.startup_deadline = None
        if not self.sidecar.available and self.feeder.automatic and not self.stopped and now >= self.retry_at:
            self.retry_at = now + 2.
            try:
                self.reconnect(wait_ready=False)
            except (OSError, RuntimeError) as error:
                logging.getLogger(__name__).warning('Equipment reconnect failed: %s', error)
        packet = self.packet
        if (not self.sidecar.available or not packet or time.monotonic()-packet['at'] >= MAX_AGE
                or self.stopped):
            self.equipment.drive(0.)
            return
        command = Commands(**packet['command'])
        if not self.synchronized:
            self.synchronized = command.phase == command.cycle == command.committed == 0
        valid = (self.synchronized and command.cycle in
                 (self.feeder.last_generated, self.feeder.last_generated + 1)
                 and time.monotonic()-packet['permit_at'] < MAX_AGE)
        sensors = Sensors()
        stock = {key: getattr(self.feeder, key) for key in
                 ('automatic', 'remaining', 'material_id', 'last_generated')}
        identity_observed = False
        body_id = ''
        try:
            sensors = self.equipment.observe(valid and packet['permitted'])
            enabled = valid and sensors.permitted and self.feeder.automatic
            if (enabled and command.feed and sensors.path_clear and sensors.pick_zone_clear
                    and not sensors.occupied):
                self.feeder.feed(command.cycle, self.equipment.spawn)
            self.equipment.step(command, sensors, dt, enabled)
            if self.equipment.cycle:
                instance = material_instance_id(self.equipment.stream_id, self.equipment.cycle)
                if self.equipment.material_path(self.equipment.material_type_id, instance):
                    body_id = instance
            identity_observed = self.synchronized
            self.failure = ''
        except (RuntimeError, ValueError) as error:
            valid = False
            self.equipment.drive(0.)
            if self.failure != str(error):
                self.failure = str(error)
                logging.getLogger(__name__).warning('Equipment observation unavailable: %s', error)
        now = time.monotonic()
        if now-self.last_sample >= .05:
            self.last_sample = now
            self.sidecar.send(dict(op='sample', at=packet['at'], acquired=now,
                                   valid=valid and self.feeder.pending is None, sensors=asdict(sensors),
                                   identity_observed=identity_observed, body_instance_id=body_id,
                                   observed=self.equipment.cycle if sensors.pick else 0,
                                   material_type_id=self.equipment.material_type_id if sensors.pick else '',
                                   created_cycle=self.feeder.last_generated,
                                   created_type_id=self.equipment.material_type_id,
                                   feeder=stock))

    def spin_once(self):
        self.sidecar.flush_outgoing()

    def spin_teaching_once(self):
        # Kit still ticks while paused. Never replay commands buffered across a pause.
        self.sidecar.poll_messages()
        self.packet = None
        self.equipment.drive(0.)

    def set_live_test_session_id(self, session_id, root=None):
        pass

    def reset_generated_materials(self, attachment_reset):
        # A timeline Stop cannot reset PLC RETAIN. Hold the stream for reconciliation.
        self.stopped = True
        self.feeder.set_automatic(False)
        self.equipment.drive(0.)
        return False

    def close(self):
        self.physics_sub = None
        clear_supply_port(self.feeder)
        try:
            self.equipment.drive(0.)
        finally:
            self.sidecar.close()


def create_adapter(*, configuration, context):
    import omni.physx
    client = ConfiguredSupplyClient.launch(Path(__file__).with_name('sidecar.py'))
    try:
        adapter = ConveyorRuntime(configuration, context['equipment'], client)
        adapter.physics_sub = omni.physx.get_physx_interface().subscribe_physics_on_step_events(
            adapter.tick, True, 0)
        return adapter
    except Exception:
        client.close()
        raise
