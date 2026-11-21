"""Exercise the GRASP and RELEASE hooks used by the physics session."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gripper_runtime.grasp_attachment import HoldingState
from gripper_runtime.surface_suction import SurfaceCapture, SurfaceManager, SuctionActuator
from gripper_runtime.tool_plugin import ToolIntent

REFERENCE = dict(strategy='surface_suction', motion_goal_uuid='a'*32, material_id='Part',
                 slot_id='cell/belt/pick', material_type_id='A', material_instance_id='run/1',
                 supply_id='belt', supply_epoch='b'*32, reservation_id='c'*32)


class Device:
    sample_value = ('Open', ())
    def sample(self):
        return self.sample_value

    def close(self):
        return True

    def open(self):
        return True


@pytest.mark.parametrize('objects', [(), ('/Part_2',), ('/Part_1', '/Part_2'), ('/Part_1',)])
def test_attachment_feedback_is_independent_of_body_identity(objects):
    device = Device()
    capture = SurfaceCapture()
    actuator = SuctionActuator(device)
    manager = SurfaceManager(actuator, tool=SimpleNamespace(capture=capture), binding=device)
    assert manager.configure_selected_grasp_reference(REFERENCE)
    actuator.command(ToolIntent.GRASP)
    assert manager.arm_close_transaction(REFERENCE['motion_goal_uuid'])
    actuator.update(.01)
    manager.update(.01)
    assert manager.holding_observation().state is HoldingState.UNKNOWN
    device.sample_value = ('Closed', objects)
    actuator.update(.01)
    manager.update(.01)
    assert manager.attachment_owned() == bool(objects)
    assert (manager.holding_observation().state is HoldingState.HELD) == bool(objects)
    # Metadata changes for the same execution cannot change the device command or feedback.
    assert manager.configure_selected_grasp_reference(dict(REFERENCE, reservation_id='d'*32))
    actuator.update(.01)
    manager.update(.01)
    assert (manager.holding_observation().state is HoldingState.HELD) == bool(objects)
    # Loss after HELD and before the release command cannot confirm placement.
    device.sample_value = ('Open', ())
    actuator.command(ToolIntent.RELEASE)
    manager.note_open_release_command_applied(2, None)
    actuator.update(.01)
    manager.update(.01)
    assert manager.release_receipt() is None

def test_grasp_without_supply_and_drop_before_release():
    device = Device()
    commands = []
    device.close = lambda: commands.append('vacuum_on')
    capture = SurfaceCapture()
    actuator = SuctionActuator(device)
    manager = SurfaceManager(actuator, tool=SimpleNamespace(capture=capture), binding=device)
    actuator.command(ToolIntent.GRASP)
    assert manager.arm_close_transaction()
    assert commands == ['vacuum_on']
    actuator.update(.01)
    manager.update(.01)
    assert manager.holding_observation().state is HoldingState.UNKNOWN
    device.sample_value = ('Closed', ('/part',))
    actuator.update(.01)
    manager.update(.01)
    assert manager.holding_observation().state is HoldingState.HELD
    device.sample_value = ('Open', ())
    actuator.update(.01)
    actuator.command(ToolIntent.RELEASE)
    actuator.update(.01)
    assert actuator.holding_observation().state is HoldingState.RELEASED
    assert not actuator.released_held_object


def test_shutdown_of_stopped_suction_tool_never_commands_grasp():
    device = Device()
    device.close = lambda: pytest.fail('shutdown commanded GRASP')
    device.reset_stopped_tool = lambda actuator: True
    capture = SurfaceCapture()
    actuator = SuctionActuator(device)
    manager = SurfaceManager(actuator, tool=SimpleNamespace(capture=capture), binding=device)
    assert manager.shutdown()
