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


def test_unreserved_arrival_loses_admission_if_the_pick_observation_is_lost():
    from threading import Lock
    from time import monotonic
    from types import SimpleNamespace
    from conveyor_pnp_cell.sidecar import Exchange

    exchange = Exchange.__new__(Exchange)
    exchange.lock = Lock()
    exchange.valid_at = monotonic()
    exchange.supply = ConveyorPickSupply('epoch')
    states = []
    exchange.publisher = SimpleNamespace(publish=states.append)
    assert exchange.supply.expose_slot('slot_12')
    for observed in (True, False, True):
        exchange.at_pick = observed
        exchange.publish_supply()
        assert states[-1].valid == observed
        assert states[-1].slot_ids == ['slot_12']


def test_sidecar_binds_observed_arrivals_beyond_the_old_teaching_range():
    from dataclasses import asdict
    from threading import Lock
    from time import monotonic
    from types import SimpleNamespace
    from conveyor_pnp_cell.equipment_io import Sensors
    from conveyor_pnp_cell.sidecar import Exchange

    exchange = Exchange.__new__(Exchange)
    exchange.lock = Lock()
    exchange.supply = ConveyorPickSupply('new-epoch')
    exchange.pick_configured = False
    samples = []
    exchange.io = SimpleNamespace(publish=lambda **sample: samples.append(sample))
    command = SimpleNamespace(cycle=65537, confirmed=True)
    packet = dict(sensors=asdict(Sensors(pick=True, stopped=True, extended=True)),
                  observed=65536, valid=True, acquired=monotonic(), feeder={})
    exchange.sample(packet, command)
    assert samples[-1]['bound_cycle'] == 0
    packet['observed'] = command.cycle
    exchange.sample(packet, command)
    assert samples[-1]['bound_cycle'] == 0
    exchange.pick_configured = True
    exchange.sample(packet, command)
    assert samples[-1]['bound_cycle'] == command.cycle
    request = ConfiguredSupply.Request(command=ConfiguredSupply.Request.RESERVE,
                                       supply_id='conveyor_pick', supply_epoch='new-epoch')
    lease = exchange.reserve(request, ConfiguredSupply.Response())
    assert lease.success and lease.slot_id == 'slot_65537'
    request.command = request.COMPLETE
    request.slot_id, request.reservation_id = lease.slot_id, lease.reservation_id
    request.material_placed = True
    assert exchange.reserve(request, ConfiguredSupply.Response()).success
    exchange.sample(packet, command)
    assert samples[-1]['bound_cycle'] == 0
