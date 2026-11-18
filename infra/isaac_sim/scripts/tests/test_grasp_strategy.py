"""Capability selection and explicit bore requirements are independent of Robot identity."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'infra/isaac_sim/scripts'))
from gripper_runtime.grasp_strategy import select_grasp_strategy
from gripper_runtime.tool_plugin import ToolIntent


def test_strategy_selection_requires_tool_capability_and_task_requirement():
    box = select_grasp_strategy({'grasp_strategies': ['box_exterior'], 'supported_shapes': ['box']},
                                {'strategy': 'box_exterior'})
    assert box.name == 'box_exterior'
    for settings, requirement in [({}, {'strategy': 'internal_radial'}),
                                   ({'grasp_strategies': ['box_exterior']}, {'strategy': 'internal_radial'}),
                                   ({'grasp_strategies': ['internal_radial']}, {'strategy': 'unknown'})]:
        with pytest.raises(ValueError):
            select_grasp_strategy(settings, requirement)


class BoreGeometry:
    def solve_bore(self, depth, thickness, radius):
        assert (depth, thickness, radius) == (.02, .002, .015)
        return dict(insertion_depth_m=depth, thickness_m=thickness, inner_radius_m=radius,
                    grasp_position_m=.005, release_position_m=.01)


def test_c10_intents_expand_for_grasp_contract_for_release_and_qualify_measured_capture():
    requirement = dict(strategy='internal_radial', insertion_depth_m=.02, thickness_m=.002, inner_radius_m=.015)
    settings = dict(grasp_strategies=['internal_radial'], position_tolerance_m=.0002,
                    sweep_duration_s=2., capabilities=['joint_position'])
    strategy=select_grasp_strategy(settings,requirement,geometry=BoreGeometry())
    assert strategy.primary_position(ToolIntent.GRASP)==.005
    assert strategy.primary_position(ToolIntent.RELEASE)==.01
    strategy.qualify_capture([0, 0, -.02], np.eye(3), .005)
    with pytest.raises(ValueError, match='expansion'):
        strategy.qualify_capture([0, 0, -.02], np.eye(3), .01)
    with pytest.raises(ValueError, match='insertion'):
        strategy.qualify_capture([0, 0, 0], np.eye(3), .005)
    with pytest.raises(ValueError):
        select_grasp_strategy(settings, {'strategy': 'internal_radial'}, geometry=BoreGeometry())


@pytest.mark.parametrize('tool_id', ['hand_e_c10', 'hand_e_c10_long'])
def test_selected_physical_binding_preserves_mimic_targets_and_rejects_limits(tool_id):
    from types import SimpleNamespace
    from arm_cell_bringup.assembly_model import load_assembly_model
    from gripper_runtime.radial_binding import IsaacRadialBinding

    settings = load_assembly_model('m0609', tool_id)['tool']['runtime']
    binding = IsaacRadialBinding.__new__(IsaacRadialBinding)
    binding.models = settings['joint_model']
    binding.dofs = {name: name for name in binding.models}
    binding.articulation = 'selected_robot'
    written = []
    def write(name, target):
        written.append((name, target))
        return True
    binding.dc = SimpleNamespace(set_dof_position_target=write, wake_up_articulation=lambda _: None)
    binding.write_primary(.0192)
    assert len(written) == 3
    assert {target for _, target in written} == {.0192}
    for target in (-.001, .02, float('nan')):
        written.clear()
        with pytest.raises(ValueError, match='limits'):
            binding.write_primary(target)
        assert written == []
