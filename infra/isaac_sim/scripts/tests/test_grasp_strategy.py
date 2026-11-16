"""Capability selection and explicit bore requirements are independent of Robot identity."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'infra/isaac_sim/scripts'))
from gripper_runtime.grasp_strategy import select_grasp_strategy
from gripper_runtime.tool_plugin import ToolIntent, radial_tool_runtime


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
    moved = []
    tool = radial_tool_runtime(lambda position: moved.append(position), {'runtime': settings},
                               BoreGeometry(), requirement)
    tool.command(ToolIntent.GRASP)
    tool.command(ToolIntent.RELEASE)
    assert moved == [.005, .01]
    tool.capture.qualify_capture([0, 0, -.02], np.eye(3), .005)
    with pytest.raises(ValueError, match='expansion'):
        tool.capture.qualify_capture([0, 0, -.02], np.eye(3), .01)
    with pytest.raises(ValueError, match='insertion'):
        tool.capture.qualify_capture([0, 0, 0], np.eye(3), .005)
    with pytest.raises(ValueError):
        select_grasp_strategy(settings, {'strategy': 'internal_radial'}, geometry=BoreGeometry())
