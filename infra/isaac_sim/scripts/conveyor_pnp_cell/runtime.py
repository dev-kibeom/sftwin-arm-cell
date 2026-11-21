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
        self.plc_offline = False
        self.diagnostics = None
        self.reservation = None
        self.retry_at = 0.
        self.startup_deadline = None
        self.synchronized = False
        self.stopped = False
        self.failure = ''
        self.last_sample = 0.
        self.sidecar_config = dict(
            op='configure', use_plc_test_epoch=True, connection=layout['connection'], permit_topic=layout['equipment_permit_topic'],
            supply=dict(cell['configured_supply'], slot_id=layout['pick_slot_id'],
                        stream_id=equipment.stream_id, material_type_ids=layout['material_type_ids']),
            pick_configured=bool(layout['teaching']['cycle']),
            receipt_file=layout.get('receipt_file'),
            history_db_path=layout.get('history_db_path', '~/.local/state/arm_cell/conveyor-pnp-poc/production.sqlite'))
        self.sidecar.send(self.sidecar_config)
        self.feeder.bind_ui(set_automatic=self.set_automatic, status=self.supply_status)

    def reconnect(self, *, wait_ready):
        self.packet = None
        self.plc_offline = False
        self.sidecar.close()
        self.sidecar = ConfiguredSupplyClient.launch(Path(__file__).with_name('sidecar.py'), wait_ready=wait_ready)
        self.sidecar.send(dict(self.sidecar_config, resume_stream=True))
        self.startup_deadline = None if wait_ready else time.monotonic() + 15.
        if not self.sidecar.available:
            raise RuntimeError('Equipment sidecar disconnected during configuration')

    def supply_status(self):
        status = self.feeder.status()
        if self.packet and time.monotonic() - self.packet['at'] < MAX_AGE:
            command = self.packet['command']
            if command.get('awaiting_receipt'):
                status['detail'] += (f" · PLC waiting for release, commit and discharge evidence "
                                     f"for robot request {command['reply_id']}")
        return status

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

    def tick(self, dt):
        now = time.monotonic()
        for packet in self.sidecar.poll_messages():
            if packet.get('op') == 'ready':
                self.startup_deadline = None
                continue
            if packet.get('op') == 'reservation':
                self.reservation = packet['reservation']
                continue
            self.packet = packet if packet.get('op') == 'command' else None
            self.plc_offline = packet.get('op') == 'offline'
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
        now = time.monotonic()
        if not self.sidecar.available:
            status = 'disconnected'
        elif packet:
            status = 'fresh' if now - packet['at'] < MAX_AGE else 'expired'
        else:
            status = 'offline' if self.plc_offline else 'not_received'
        if status != 'fresh' or self.stopped:
            self.equipment.drive(0.)
            self._record_decision(now, status)
            return
        command = Commands(**packet['command'])
        if not self.synchronized:
            self.synchronized = command.phase == command.cycle == command.committed == 0
        cycle_matches = command.cycle in (self.feeder.last_generated, self.feeder.last_generated + 1)
        permit_fresh = now - packet['permit_at'] < MAX_AGE
        valid = self.synchronized and cycle_matches and permit_fresh
        sensors = Sensors()
        stock = {key: getattr(self.feeder, key) for key in
                 ('automatic', 'remaining', 'material_id', 'last_generated')}
        pending_before = self.feeder.pending
        observed_sensors = None
        identity_observed = False
        body_id = ''
        try:
            sensors = self.equipment.observe(permit_fresh and packet['permitted'])
            observed_sensors = sensors
            enabled = valid and sensors.permitted and self.feeder.automatic
            if enabled and command.feed:
                self.feeder.feed(command.cycle, self.equipment.spawn)
            self.equipment.step(command, sensors, dt, self.synchronized and sensors.permitted)
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
        if self.diagnostics is not None:
            self._record_decision(now, status, feeder=dict(stock, pending=pending_before),
                                  cycle_matches=cycle_matches, permit_fresh=permit_fresh,
                                  sensors=observed_sensors, error=self.failure or None)
        now = time.monotonic()
        if now-self.last_sample >= .05:
            self.last_sample = now
            self.sidecar.send(dict(op='sample', at=packet['at'], acquired=now,
                                   valid=valid and self.feeder.pending is None, sensors=asdict(sensors),
                                   identity_observed=identity_observed, body_instance_id=body_id,
                                   observed=self.equipment.cycle if sensors.pick else 0,
                                   material_type_id=self.equipment.material_type_id if sensors.pick else '',
                                   last_discharge=getattr(self.equipment, 'last_discharge', None),
                                   created_cycle=self.feeder.last_generated,
                                   created_type_id=self.equipment.material_type_id,
                                   feeder=stock))

    def spin_once(self):
        self.sidecar.flush_outgoing()

    def spin_teaching_once(self):
        # Kit still ticks while paused. Never replay commands buffered across a pause.
        self.sidecar.poll_messages()
        self.packet = None
        self.plc_offline = False
        self.equipment.drive(0.)

    def _record_decision(self, now, status, *, feeder=None, **inputs):
        if self.diagnostics is None:
            return
        try:
            if feeder is None:
                feeder = {key: getattr(self.feeder, key) for key in
                          ('automatic', 'remaining', 'material_id', 'last_generated', 'pending')}
            self.diagnostics.observe(stream_id=self.equipment.stream_id, now=now, command_status=status,
                                     packet=self.packet, synchronized=self.synchronized, feeder=feeder,
                                     stopped=self.stopped, max_age=MAX_AGE, **inputs)
        except Exception as error:
            logging.getLogger(__name__).warning('Conveyor decision recording unavailable: %s', error)
            self.set_live_test_session_id(None)

    def set_live_test_session_id(self, session_id, root=None):
        previous, self.diagnostics = self.diagnostics, None
        try:
            if previous is not None:
                previous.close()
            if session_id is not None and root is not None:
                from .feed_diagnostics import FeedDiagnostics
                self.diagnostics = FeedDiagnostics(session_id, root)
        except Exception as error:
            logging.getLogger(__name__).warning('Conveyor decision recorder unavailable: %s', error)

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
            try:
                self.sidecar.close()
            finally:
                self.set_live_test_session_id(None)


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
