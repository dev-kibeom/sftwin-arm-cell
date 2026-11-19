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
    radial = RadialPhysicalTool(Strategy(), lambda q: writes.append(q), lambda: values[0],
        lower=0., upper=.0192, speed_m_s=.01, preload_m=.001,
        tolerance_m=.0002, dwell_s=.1)
    return radial, values, writes


def test_obstruction_is_inferred_from_primary_position_without_contact_truth():
    radial, values, _ = tool()
    radial.command(ToolIntent.GRASP)
    values[:] = [.009]*3
    radial.read_contacts=lambda dt: (_ for _ in ()).throw(AssertionError('contact truth queried'))
    for _ in range(150): radial.update(.01)
    assert radial.holding_observation().state.value == 'held'
    # Reaching the commanded endpoint is an empty close, not possession.
    values[:] = [.008]*3
    for _ in range(150): radial.update(.01)
    assert radial.holding_observation().state.value == 'unknown'


def test_release_uses_measured_contraction_without_contact_truth():
    radial, values, _ = tool()
    radial.command(ToolIntent.RELEASE)
    values[:] = [.009]*3
    for _ in range(150): radial.update(.01)
    assert radial.holding_observation().state.value == 'unknown'
    values[:] = [.0192]*3
    for _ in range(150): radial.update(.01)
    assert radial.holding_observation().state.value == 'released'


def test_stop_holds_measured_position_without_releasing_or_teleporting():
    radial, values, writes = tool()
    radial.command(ToolIntent.GRASP)
    radial.update(.01)
    values[:] = [.012]*3
    radial.command(ToolIntent.STOP)
    assert writes[-1] == pytest.approx(.012)
    radial.update(.01)
    assert writes[-1] == pytest.approx(.012)
    assert radial.holding_observation().state.value == 'unknown'


def test_loss_of_position_stability_invalidates_holding():
    radial, values, _ = tool()
    radial.command(ToolIntent.GRASP)
    values[:] = [.009]*3
    for _ in range(150): radial.update(.01)
    values[:] = [.010]*3
    radial.update(.01)
    assert radial.holding_observation().state.value == 'unknown'


def test_stop_near_joint_limit_uses_a_legal_hold_target():
    radial, values, writes = tool()
    values[:] = [.01920001]*3  # finite PhysX solver residual, not a new limit
    radial.command(ToolIntent.STOP)
    assert writes[-1] == .0192



def test_stopped_actuator_stays_suspended_until_new_binding_on_play():
    radial, _, _ = tool()
    radial.reset_stopped_state()
    radial.measure = lambda: (_ for _ in ()).throw(RuntimeError('old DC handle'))
    radial.update(.01)
    assert radial.observation()['primary_position_m'] is None
    assert radial.holding_observation().state.value == 'unknown'
    with pytest.raises(ValueError):
        radial.command(ToolIntent.GRASP)


def test_long_contact_links_follow_fixed_extensions_in_measured_joint_order():
    import xml.etree.ElementTree as ET
    from gripper_runtime.radial_binding import contact_links_by_joint
    urdf=ET.fromstring('<robot>'+''.join(
        f'<joint name="j{i}" type="prismatic"><parent link="base"/><child link="slider{i}"/></joint>'
        f'<joint name="ext{i}" type="fixed"><parent link="slider{i}"/><child link="tip{i}"/></joint>'
        for i in range(1,4))+'</robot>')
    assert contact_links_by_joint(urdf,['j3','j1','j2'],['tip1','tip2','tip3'])==['tip3','tip1','tip2']
    assert contact_links_by_joint(urdf,['j1','j2','j3'],['slider3','slider2','slider1'])==['slider1','slider2','slider3']
    with pytest.raises(ValueError):contact_links_by_joint(urdf,['j1','j2','j3'],['tip1','tip2','base'])
    urdf.find("joint[@name='ext3']").set('type','revolute')
    with pytest.raises(ValueError):contact_links_by_joint(urdf,['j1','j2','j3'],['tip1','tip2','tip3'])


def test_slow_continuous_motion_is_not_stable_possession():
    radial, values, _ = tool()
    radial.command(ToolIntent.GRASP)
    radial.target=.008
    values[:] = [.009]*3
    for _ in range(100):
        values[0]+=.0001
        radial.update(.01)
    assert radial.holding_observation().state.value=='unknown'
