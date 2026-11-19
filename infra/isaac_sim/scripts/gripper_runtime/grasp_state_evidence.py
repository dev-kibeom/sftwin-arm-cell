"""Bounded physical evidence around possession transitions, never control input."""
from collections import deque
import time


class GraspStateEvidence:
    def __init__(self, submit, observe, *, detailed=False, max_window=8):
        self.submit, self.observe = submit, observe
        self.detailed = detailed
        self.before = deque(maxlen=max_window)
        self.window = max_window
        self.elapsed = 0.
        self.identity = self.state = None
        self.remaining = 0

    def update(self, dt, execution_id, holding_state, physics_step):
        self.elapsed += max(0., dt)
        if not execution_id:
            self.before.clear()
            self.identity = self.state = None
            return
        changed = (execution_id, holding_state) != (self.identity, self.state)
        if not self.detailed and not changed:
            return
        if not changed and self.elapsed < .2:
            return
        self.elapsed = 0.
        try:
            observation = self.observe()
        except Exception as error:
            observation = dict(status='unavailable', reason=f'{type(error).__name__}: {error}')
        sample = dict(observed_monotonic_ns=time.monotonic_ns(), physics_step=physics_step,
                      observation=observation)
        if execution_id != self.identity:
            self.before.clear()
        event = 'holding_changed' if changed else 'post_event'
        if changed or (self.detailed and self.remaining > 0):
            self.submit(dict(schema_version=1, record_type='grasp_state_evidence',
                record_id=f'grasp-state-{time.monotonic_ns()}', execution_id=execution_id,
                time_basis='isaac_local_monotonic', frame_id='world', event=event,
                holding_state=holding_state, **sample,
                before=list(self.before) if changed and self.detailed else [],
                coverage=dict(pre_event_samples=len(self.before) if self.detailed else 0,
                              window_capacity=self.window, observation_interval_sim_s=.2,
                              physics_step_scope='gripper_runtime_session_callback')))
        self.remaining = self.window if changed else max(0, self.remaining - 1)
        self.identity, self.state = execution_id, holding_state
        if self.detailed:
            self.before.append(sample)
