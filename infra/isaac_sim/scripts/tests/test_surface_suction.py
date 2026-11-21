"""Exercise the same reservation, GRASP and RELEASE hooks used by the physics session."""
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
def test_only_reserved_single_body_can_be_held(objects):
    device = Device()
    capture = SurfaceCapture()
    capture.resolve_reserved_body = lambda ref: '/Part_1' if ref == REFERENCE else None
    actuator = SuctionActuator(device, capture)
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
    assert (manager.holding_observation().state is HoldingState.HELD) == (objects == ('/Part_1',))
    # Loss after HELD and before the release command cannot confirm placement.
    device.sample_value = ('Open', ())
    actuator.command(ToolIntent.RELEASE)
    manager.note_open_release_command_applied(2, None)
    actuator.update(.01)
    manager.update(.01)
    assert manager.release_receipt() is None

def test_expired_or_replaced_reservation_cannot_reconfirm_a_dropped_object():
    current = [REFERENCE]
    device = Device()
    capture = SurfaceCapture()
    capture.resolve_reserved_body = lambda ref: '/part' if ref == current[0] else None
    actuator = SuctionActuator(device, capture)
    actuator.strategy = REFERENCE
    actuator.command(ToolIntent.GRASP)
    device.sample_value = ('Closed', ('/part',))
    actuator.update(.01)
    assert actuator.holding_observation().state is HoldingState.HELD
    current[0] = {**REFERENCE, 'reservation_id': 'd'*32}
    actuator.update(.01)
    assert actuator.holding_observation().state is HoldingState.UNKNOWN
    current[0] = REFERENCE
    actuator.update(.01)
    assert actuator.holding_observation().state is HoldingState.UNKNOWN
    actuator.command(ToolIntent.RELEASE)
    device.sample_value = ('Open', ())
    actuator.update(.01)
    assert actuator.holding_observation().state is HoldingState.UNKNOWN
    actuator.strategy = {**REFERENCE, 'motion_goal_uuid': 'e'*32}
    actuator.command(ToolIntent.RELEASE)
    actuator.update(.01)
    assert actuator.holding_observation().state is HoldingState.RELEASED
    assert not actuator.released_target
