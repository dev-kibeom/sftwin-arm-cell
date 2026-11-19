"""Independent Ground Truth verdicts; no result is returned to process control."""
import time


class CycleEvidence:
    def __init__(self, submit, session_id=None):
        self.submit, self.session_id = submit, session_id
        self.active = None

    def start(self, cycle_id, **identity):
        if self.active is not None:
            self.finish(False)
        self.active = dict(schema_version=1, record_type='tape_cycle_verification',
            record_id=cycle_id, cycle_id=cycle_id, session_id=self.session_id,
            time_basis='isaac_local_monotonic', started_monotonic_ns=time.monotonic_ns(),
            observed_steps=0, coverage_complete=True, collisions=[], collision_count=0,
            **identity)

    def observe(self, collisions, *, physics_step=True):
        record = self.active
        if record is None:
            return
        if collisions is None:
            record['coverage_complete'] = False
            return
        if physics_step:
            record['observed_steps'] += 1
        record['collision_count'] += len(collisions)
        record['collisions'].extend(collisions[:max(0, 32-len(record['collisions']))])

    def finish(self, sequence_completed):
        record, self.active = self.active, None
        if record is None:
            return
        record.update(sequence_completed=bool(sequence_completed),
                      ended_monotonic_ns=time.monotonic_ns())
        complete = record['coverage_complete'] and record['observed_steps'] > 0
        record['verification'] = ('FAIL' if not sequence_completed or record['collision_count'] else
                                  'PASS' if complete else 'UNAVAILABLE')
        record['coverage'] = dict(contact_observation_complete=complete,
            collision_details_truncated=record['collision_count'] > len(record['collisions']))
        try:
            self.submit(record)
        except Exception:
            # A broken evidence sink must not change Motion, Supply or Mission.
            pass
