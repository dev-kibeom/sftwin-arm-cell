"""Contact evidence, not commanded position, controls radial Holding."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gripper_runtime.radial_physical import RadialPhysicalTool
from gripper_runtime.tool_plugin import ToolIntent


class Strategy:
    name = 'internal_radial'
    solution = dict(grasp_position_m=.009, release_position_m=.0192)
    def primary_position(self, intent):
        return self.solution['grasp_position_m' if intent == 'GRASP' else 'release_position_m']


def tool():
    values = [.0192]*3
    writes = []
    radial = RadialPhysicalTool(Strategy(), lambda q: writes.append(q), lambda: values,
        lower=0., upper=.0192, speed_m_s=.01, preload_m=.001,
        tolerance_m=.0002, contact_threshold_n=.2, dwell_s=.1)
    return radial, values, writes


def test_reached_target_and_unrelated_contact_do_not_mean_held():
    radial, values, writes = tool()
    radial.command(ToolIntent.GRASP)
    values[:] = [.009]*3
    for _ in range(150):
        radial.update(.01, [0,0,0])
    assert radial.holding_observation().state.value == 'unknown'
    for _ in range(150):
        radial.update(.01, [1,1,1])
    assert radial.holding_observation().state.value == 'held'
    radial.update(.01, [0,1,1])
    assert radial.holding_observation().state.value == 'unknown'


def test_release_needs_measured_contraction_and_no_contact():
    radial, values, writes = tool()
    radial.command(ToolIntent.RELEASE)
    values[:] = [.009]*3
    for _ in range(150): radial.update(.01, [0,0,0])
    assert radial.holding_observation().state.value == 'unknown'
    values[:] = [.0192]*3
    for _ in range(150): radial.update(.01, [1,1,1])
    assert radial.holding_observation().state.value == 'unknown'
    for _ in range(150): radial.update(.01, [0,0,0])
    assert radial.holding_observation().state.value == 'released'


def test_stop_holds_measured_position_without_releasing_or_teleporting():
    radial, values, writes = tool()
    radial.command(ToolIntent.GRASP)
    radial.update(.01, [0,0,0])
    values[:] = [.012]*3
    radial.command(ToolIntent.STOP)
    assert writes[-1] == pytest.approx(.012)
    radial.update(.01, [0,0,0])
    assert writes[-1] == pytest.approx(.012)
    assert radial.holding_observation().state.value == 'unknown'


def test_missing_or_nonfinite_contacts_cannot_reuse_held_evidence():
    radial, values, _ = tool()
    radial.command(ToolIntent.GRASP)
    values[:] = [.009]*3
    for _ in range(150): radial.update(.01, [1,1,1])
    radial.update(.01, None)
    assert radial.holding_observation().state.value == 'unknown'
    radial.update(.01, [float('nan')]*3)
    assert radial.holding_observation().state.value == 'unknown'


def test_stop_near_joint_limit_uses_a_legal_hold_target():
    radial, values, writes = tool()
    values[:] = [.01920001]*3  # finite PhysX solver residual, not a new limit
    radial.command(ToolIntent.STOP)
    assert writes[-1] == .0192
