"""Physical observations remain independent of device possession decisions."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gripper_runtime.grasp_state_evidence import GraspStateEvidence


def test_idle_does_not_sample_or_write_and_late_loss_keeps_pre_and_post_window():
    records, observations = [], []
    def observe():
        observations.append(len(observations))
        return {'materials': [{'prim_path': '/roll', 'position_m': [0, 0, len(observations)]}]}
    evidence = GraspStateEvidence(records.append, observe, detailed=True, max_window=4)
    for _ in range(100):
        evidence.update(.2, None, 'unknown', 0)
    assert not records and not observations
    for step in range(100):
        evidence.update(.2, 'goal', 'held', step)
    evidence.update(.2, 'goal', 'unknown', 100)
    loss = records[-1]
    assert loss['event'] == 'holding_changed'
    assert len(loss['before']) == 4
    assert loss['observation']['materials'][0]['prim_path'] == '/roll'
    for step in range(101, 110):
        evidence.update(.2, 'goal', 'unknown', step)
    assert any(record['event'] == 'post_event' for record in records)
    assert len(records) < 30


def test_observation_failure_is_recorded_and_does_not_escape():
    records = []
    def broken():
        raise RuntimeError('PhysX unavailable')
    evidence = GraspStateEvidence(records.append, broken)
    evidence.update(.2, 'goal', 'held', 3)
    assert records[0]['observation']['status'] == 'unavailable'
    assert records[0]['execution_id'] == 'goal'


def test_tcp_state_uses_fixed_offset_and_angular_velocity_at_same_body_observation():
    import math
    from isaac_runtime.material_observations import tcp_state_from_body
    state = {'status': 'observed', 'prim_path': '/body', 'position_m': [1, 2, 3],
             'orientation_xyzw': [0, 0, math.sqrt(.5), math.sqrt(.5)],
             'linear_velocity_m_s': [1, 0, 0], 'angular_velocity_rad_s': [0, 0, 2]}
    tcp = tcp_state_from_body(state, [1, 0, 0, 0, 0, 0, 1], '/tcp')
    import pytest
    assert tcp['position_m'] == pytest.approx([1, 3, 3])
    assert tcp['linear_velocity_m_s'] == pytest.approx([-1, 0, 0])
    assert tcp['rigid_body_prim_path'] == '/body' and tcp['prim_path'] == '/tcp'
    assert tcp_state_from_body({'status': 'unavailable'}, [], '/tcp')['status'] == 'unavailable'


def test_dynamic_observer_keeps_selected_identity_when_material_budget_is_exceeded():
    from isaac_runtime.material_observations import DynamicBodyObserver
    observer = object.__new__(DynamicBodyObserver)
    observer.body = lambda path: {'status': 'observed', 'prim_path': path}
    observer.tcp = observer.body
    targets = [{'prim_path': f'/material/{i}', 'selected': i == 20} for i in range(21)]
    snapshot = observer.snapshot('/tcp', targets, max_materials=2)
    assert snapshot['materials'][0]['identity']['prim_path'] == '/material/20'
    assert snapshot['coverage']['omitted_material_count'] == 19


def test_lightweight_mode_reads_truth_only_at_state_changes():
    samples = []
    evidence = GraspStateEvidence(lambda record: None, lambda: samples.append(1) or {})
    for step in range(100):
        evidence.update(.2, 'goal', 'held', step)
    evidence.update(.2, 'goal', 'unknown', 100)
    assert len(samples) == 2
