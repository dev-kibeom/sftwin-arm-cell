"""Transition evidence from Isaac's feed decision, scoped to one live session."""
from pathlib import Path

from gripper_runtime.capture_diagnostics import BoundedCaptureDiagnosticWriter


class FeedDiagnostics:
    def __init__(self, session_id, root):
        self.session_id = session_id
        self.writer = BoundedCaptureDiagnosticWriter(Path(root) / 'simulation' / 'conveyor_supply', max_pending=64)
        self.previous = None
        self.sequence = 0

    def observe(self, *, stream_id, now, command_status, packet, synchronized, feeder, stopped,
                max_age, cycle_matches=None, permit_fresh=None, sensors=None, error=None):
        """Snapshot the inputs used on this step. Sample ages alone are not transitions."""
        plc = None
        command_age = permit_age = permit_value = None
        permit_status = 'unknown'
        blockers = []
        if packet:
            command_age = now - packet['at']
            plc = {key: packet['command'][key] for key in ('phase', 'cycle', 'committed', 'feed')}
            if packet['command'].get('requested') is not None:
                plc.update({key: packet['command'][key] for key in ('requested', 'reply_id', 'reply_state')})
                if packet['command']['awaiting_receipt']:
                    blockers.append('previous_robot_request_unreconciled')
            if packet['permit_at']:
                permit_age = now - packet['permit_at']
                permit_value = packet['permitted']
                permit_status = 'fresh' if permit_age < max_age else 'expired'
            else:
                permit_status = 'not_received'

        if stopped:
            blockers.append('timeline_stopped')
        if command_status != 'fresh':
            blockers.append('command_' + command_status)
        elif not stopped:
            if not synchronized:
                blockers.append('awaiting_initial_plc_state')
            if not cycle_matches:
                blockers.append('cycle_mismatch')
            if not permit_fresh:
                blockers.append('permit_' + permit_status)
            elif not permit_value:
                blockers.append('permit_denied')
            if not plc['feed']:
                blockers.append('no_feed_command')
            elif plc['cycle'] != feeder['last_generated'] + 1:
                blockers.append('not_next_material')
        if not feeder['automatic']:
            blockers.append('automatic_paused')
        if not feeder['remaining']:
            blockers.append('no_stock')
        if feeder['pending'] is not None:
            blockers.append('creation_pending')
        if error:
            blockers.append('equipment_error')

        state = dict(command_status=command_status, plc=plc, synchronized=synchronized,
                     permit_status=permit_status, permit_value=permit_value, **feeder,
                     path_clear=None if sensors is None else sensors.path_clear,
                     pick_zone_clear=None if sensors is None else sensors.pick_zone_clear,
                     occupied=None if sensors is None else sensors.occupied, blockers=blockers, error=error)
        if state == self.previous:
            return
        self.previous = state
        self.sequence += 1
        self.writer.submit(dict(schema_version=1, record_type='conveyor_feed_decision',
                                record_id=f'decision-{self.sequence:08d}', live_test_session_id=self.session_id,
                                stream_id=stream_id, observed_monotonic_s=now, command_age_s=command_age,
                                permit_age_s=permit_age, **state))

    def close(self):
        self.writer.close()
