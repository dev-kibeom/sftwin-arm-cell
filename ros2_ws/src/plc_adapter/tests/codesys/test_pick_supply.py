"""Arrival and reservation identity on the Conveyor equipment side of the boundary."""
from pathlib import Path
import sys

from arm_cell_interfaces.srv import ConfiguredSupply
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[5] / 'infra/isaac_sim/scripts'))
from conveyor_pnp_cell.configured_supply import ConveyorPickSupply


@pytest.mark.parametrize('command', [ConfiguredSupply.Request.COMPLETE, ConfiguredSupply.Request.ABANDON])
def test_release_removes_only_the_reserved_arrival_even_after_failure(command):
    supply = ConveyorPickSupply('epoch')
    assert supply.expose_slot('slot_12')

    def call(command, *, slot='', reservation='', placed=False, epoch='epoch', valid=True):
        return supply.handle(ConfiguredSupply.Request(
            command=command, supply_id='conveyor_pick', supply_epoch=epoch, slot_id=slot,
            reservation_id=reservation, material_placed=placed), ConfiguredSupply.Response(), valid=valid)

    reserve = ConfiguredSupply.Request.RESERVE
    lease = call(reserve)
    assert lease.success and lease.slot_id == 'slot_12'
    assert not call(reserve).success
    assert not supply.expose_slot('slot_12') and not supply.expose_slot('slot_13')
    assert not call(command, slot=lease.slot_id, reservation=lease.reservation_id,
                    placed=True, epoch='old').success
    assert call(command, slot=lease.slot_id, reservation=lease.reservation_id, placed=True, valid=False).success
    assert not supply.expose_slot('slot_12') and call(reserve).exhausted
    assert supply.expose_slot('slot_13')
    second = call(reserve)
    assert second.success and second.slot_id == 'slot_13'
    assert not call(command, slot=lease.slot_id, reservation=lease.reservation_id, placed=True).success
    assert call(ConfiguredSupply.Request.ABANDON, slot=second.slot_id, reservation=second.reservation_id).success
    assert call(reserve).slot_id == 'slot_13'
