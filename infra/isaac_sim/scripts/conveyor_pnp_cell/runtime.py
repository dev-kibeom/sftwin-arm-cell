"""Isaac owns the physical receipt and advances the equipment only on simulation steps."""
from dataclasses import asdict
from pathlib import Path
import logging
import time

from isaac_runtime.cell_supply import clear_supply_port
from tape_roll_cell.configured_supply_sidecar import ConfiguredSupplyClient
from .equipment_io import Commands, Sensors
from .supply import ConveyorFeeder

MAX_AGE = .9


class ConveyorRuntime:
    def __init__(self, configuration, equipment, sidecar):
        self.equipment, self.sidecar = equipment, sidecar
        cell = configuration['cell']
        layout = cell['conveyor']
        self.feeder = ConveyorFeeder(cell['configured_supply']['material_id'], last_generated=0,
                                     material_type_ids=layout['material_type_ids'])
        self.packet = None
        self.synchronized = False
        self.stopped = False
        self.failure = ''
        self.last_sample = 0.
        self.sidecar.send(dict(op='configure', connection=cell['conveyor']['connection'],
                               permit_topic=cell['conveyor']['equipment_permit_topic'],
                               supply=dict(cell['configured_supply'], slot_id=layout['pick_slot_id'],
                                           stream_id=equipment.stream_id,
                                           material_type_ids=layout['material_type_ids']),
                               pick_configured=bool(cell['conveyor']['teaching']['cycle'])))
        self.feeder.bind_ui()

    def set_tool_session(self, session):
        session.tool.capture.resolve_reserved_body = self.reserved_body

    def reserved_body(self, reference):
        packet = self.packet
        if (self.stopped or not self.sidecar.available or not packet
                or time.monotonic() - packet['at'] >= MAX_AGE):
            return None
        reservation = packet.get('reservation')
        if not reservation or any(reference.get(key) != reservation.get(key) for key in (
                'material_id', 'supply_id', 'slot_id', 'supply_epoch', 'reservation_id',
                'material_type_id', 'material_instance_id')):
            return None
        return self.equipment.material_path(reference['material_type_id'], reference['material_instance_id'])

    def tick(self, dt):
        for packet in self.sidecar.poll_messages():
            self.packet = packet if packet.get('op') == 'command' else None
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
        try:
            sensors = self.equipment.observe(valid and packet['permitted'])
            enabled = valid and sensors.permitted and self.feeder.automatic
            if (enabled and command.feed and sensors.path_clear and sensors.pick_zone_clear
                    and not sensors.occupied):
                self.feeder.feed(command.cycle, self.equipment.spawn)
            self.equipment.step(command, sensors, dt, enabled)
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
                                   observed=self.equipment.cycle if sensors.pick else 0,
                                   material_type_id=self.equipment.material_type_id if sensors.pick else '',
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
