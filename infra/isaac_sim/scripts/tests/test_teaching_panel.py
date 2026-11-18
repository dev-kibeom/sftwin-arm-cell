"""Operator inputs are converted at the teaching owner command boundary."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


SOURCE = (Path(__file__).resolve().parents[2] / 'extensions/arm_cell.simulation_ui_hub/python'
          / 'arm_cell_simulation_ui_hub/teaching_panel.py')
spec = importlib.util.spec_from_file_location('teaching_panel', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def field(value):
    return SimpleNamespace(model=SimpleNamespace(as_float=value))


def test_point_update_submits_operator_units_as_metres_radians_and_scales():
    panel = module.TeachingPanel(None)
    selected = {'point': 'insert', 'point_frame': 'infeed', 'mode': 'linear'}
    panel._selected = selected.get
    panel._point_pose = [field(v) for v in [100, -200, 300, 90, -45, 180]]
    panel._speed = field(25)
    panel._acceleration = field(10)
    commands = []
    panel._command = lambda operation, **values: commands.append((operation, values))
    panel._edit_point()
    operation, values = commands[0]
    assert operation == 'edit_point'
    assert values['name'] == 'insert'
    point = values['point']
    assert point['frame'] == 'infeed'
    assert point['mode'] == 'linear'
    assert point['pose'] == pytest.approx([.1, -.2, .3, math.pi / 2, -math.pi / 4, math.pi])
    assert point['velocity_scale'] == .25
    assert point['acceleration_scale'] == .1


@pytest.mark.parametrize('axis,sign,expected', [(0, -1, -.002), (5, 1, math.pi / 180)])
def test_jog_converts_selected_step_in_selected_frame(axis, sign, expected):
    panel = module.TeachingPanel(None)
    panel._selected = lambda _: 'tool'
    panel._linear_step = field(2)
    panel._angular_step = field(1)
    commands = []
    panel._command = lambda operation, **values: commands.append((operation, values))
    panel._jog(axis, sign)
    assert commands[0][0] == 'jog'
    assert commands[0][1] == {'frame': 'tool', 'axis': axis, 'step': pytest.approx(expected)}


def test_sequence_reorder_does_not_mutate_owner_status_before_acceptance():
    panel = module.TeachingPanel(None)
    original = ['approach', 'insert', 'retreat']
    panel._status = {'program': {'sequences': {'tape_insert': original}}}
    panel._choices['sequence_step'] = ['1: approach', '2: insert', '3: retreat']
    panel._selected = {'sequence': 'tape_insert', 'sequence_step': '2: insert'}.get
    commands = []
    panel._command = lambda operation, **values: commands.append((operation, values))
    panel._edit_sequence('up')
    assert commands == [('edit_sequence', {'name': 'tape_insert', 'points': ['insert', 'approach', 'retreat']})]
    assert original == ['approach', 'insert', 'retreat']
