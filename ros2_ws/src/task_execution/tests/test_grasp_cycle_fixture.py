"""Native ABI failure containment, exercised without an Isaac dependency."""
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent / 'support'))
from grasp_cycle_fixture import run_task_grasp_cycle


def test_native_task_uses_semantic_ports_and_stops_on_callback_failure():
    events = []
    holding = 0
    def callback(event, name):
        nonlocal holding
        events.append((event, name))
        if event == 1:
            holding = 1
        elif event == 2:
            holding = 2
        elif event == 3:
            return holding
        return 0
    solution = dict(insertion_depth_m=.02, thickness_m=.002, inner_radius_m=.025,
                    release_position_m=.02, grasp_position_m=.01)
    provenance = run_task_grasp_cycle(os.environ['ARM_CELL_GRASP_FIXTURE_LIBRARY'],
                                      'internal_radial', solution, callback)
    assert provenance['result_code'] == 0
    assert [(event, name) for event, name in events if event in (0, 1, 2)] == [
        (2, ''), (0, 'approach'), (0, 'insert'), (1, ''),
        (0, 'transfer'), (2, ''), (0, 'retreat')]
    events.clear()
    def failing(event, name):
        if event == 0 and name == 'insert':
            raise RuntimeError('Live fixture insertion failed')
        return callback(event, name)
    with pytest.raises(RuntimeError, match='Live fixture insertion failed'):
        run_task_grasp_cycle(os.environ['ARM_CELL_GRASP_FIXTURE_LIBRARY'],
                             'internal_radial', solution, failing)
    assert (1, '') not in events
    assert (0, 'transfer') not in events
