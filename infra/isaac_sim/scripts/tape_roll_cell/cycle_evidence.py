"""Independent Ground Truth verdicts; no result is returned to process control."""
import time


class CycleEvidence:
    def __init__(self, submit, session_id=None):
        self.submit, self.session_id = submit, session_id
        self.active = None
        self._pending_step = None

    def start(self, cycle_id, **identity):
        if self.active is not None:
            self.finish(False)
        self._pending_step = None
        self.active = dict(schema_version=1, record_type='tape_cycle_verification',
            record_id=cycle_id, cycle_id=cycle_id, session_id=self.session_id,
            time_basis='isaac_local_monotonic', started_monotonic_ns=time.monotonic_ns(),
            executed_steps=0, observed_steps=0, first_step_id=None,
            last_executed_step_id=None, last_observed_step_id=None,
            coverage_complete=True, collisions=[], collision_count=0,
            **identity)

    def physics_step(self, step_id):
        """Register execution independently of the contact observation callback."""
        record = self.active
        if record is None:
            return
        record['executed_steps'] += 1
        previous = record['last_executed_step_id']
        if self._pending_step is not None:
            record['coverage_complete'] = False
        if not isinstance(step_id, int) or isinstance(step_id, bool) or step_id <= 0:
            record['coverage_complete'] = False
            return
        if previous is not None and step_id != previous+1:
            record['coverage_complete'] = False
        if record['first_step_id'] is None:
            record['first_step_id'] = step_id
        record['last_executed_step_id'] = step_id
        self._pending_step = step_id

    def observe(self, collisions, *, step_id=None, physics_step=True):
        record = self.active
        if record is None:
            return
        if collisions is None:
            record['coverage_complete'] = False
            return
        if physics_step:
            if self._pending_step is not None and step_id == self._pending_step:
                record['observed_steps'] += 1
                record['last_observed_step_id'] = step_id
                self._pending_step = None
            else:
                record['coverage_complete'] = False
        record['collision_count'] += len(collisions)
        record['collisions'].extend(collisions[:max(0, 32-len(record['collisions']))])

    def finish(self, sequence_completed):
        record, self.active = self.active, None
        if record is None:
            return
        record.update(sequence_completed=bool(sequence_completed),
                      ended_monotonic_ns=time.monotonic_ns())
        complete = (record['coverage_complete'] and record['executed_steps'] > 0
                    and record['observed_steps'] == record['executed_steps']
                    and self._pending_step is None)
        self._pending_step = None
        record['coverage_complete'] = complete
        record['verification'] = ('FAIL' if not sequence_completed or record['collision_count'] else
                                  'PASS' if complete else 'UNAVAILABLE')
        record['coverage'] = dict(contact_observation_complete=complete,
            collision_details_truncated=record['collision_count'] > len(record['collisions']))
        try:
            self.submit(record)
        except Exception:
            # A broken evidence sink must not change Motion, Supply or Mission.
            pass
