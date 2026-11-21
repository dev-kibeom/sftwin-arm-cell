"""Arrival and reservation identity on the Conveyor equipment side of the boundary."""
from pathlib import Path
import sys

from arm_cell_interfaces.srv import ConfiguredSupply
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[5] / 'infra/isaac_sim/scripts'))
from conveyor_pnp_cell.configured_supply import ConveyorPickSupply


def pick_supply(epoch, stream_id='test'):
    return ConveyorPickSupply(epoch, supply_id='conveyor_pick', slot_id='cell/belt/pick',
                              stream_id=stream_id, material_id='ConveyorPart', material_type_ids=['A', 'B'])


@pytest.mark.parametrize('command', [ConfiguredSupply.Request.COMPLETE, ConfiguredSupply.Request.ABANDON])
def test_release_removes_only_the_reserved_arrival_even_after_failure(command):
    supply = pick_supply('epoch')
    assert supply.expose_arrival(12, 'A')

    def call(command, *, slot='', reservation='', instance='test/12', material_type='A',
             placed=False, epoch='epoch', valid=True):
        return supply.handle(ConfiguredSupply.Request(
            command=command, supply_id='conveyor_pick', supply_epoch=epoch, slot_id=slot,
            reservation_id=reservation, material_instance_id=instance, material_type_id=material_type,
            material_placed=placed), ConfiguredSupply.Response(), valid=valid)

    reserve = ConfiguredSupply.Request.RESERVE
    lease = call(reserve)
    assert lease.success and lease.slot_id == 'cell/belt/pick'
    assert not call(reserve).success
    assert not supply.expose_arrival(12, 'A') and not supply.expose_arrival(13, 'B')
    assert not call(command, slot=lease.slot_id, reservation=lease.reservation_id,
                    placed=True, epoch='old').success
    assert call(command, slot=lease.slot_id, reservation=lease.reservation_id, placed=True, valid=False).success
    assert not supply.expose_arrival(12, 'A') and call(reserve).exhausted
    assert supply.expose_arrival(13, 'B')
    second = call(reserve)
    assert (second.material_type_id, second.material_instance_id) == ('B', 'test/13')
    assert not call(command, slot=second.slot_id, reservation=second.reservation_id, placed=True).success
    assert second.success and second.slot_id == 'cell/belt/pick'
    assert not call(command, slot=lease.slot_id, reservation=lease.reservation_id, placed=True).success
    assert call(ConfiguredSupply.Request.ABANDON, slot=second.slot_id,
                reservation=second.reservation_id, instance='test/13', material_type='B').success
    assert call(reserve).slot_id == 'cell/belt/pick'


def test_unreserved_arrival_loses_admission_if_the_pick_observation_is_lost():
    from threading import Lock
    from time import monotonic
    from types import SimpleNamespace
    from conveyor_pnp_cell.sidecar import Exchange

    exchange = Exchange.__new__(Exchange)
    exchange.lock = Lock()
    exchange.valid_at = monotonic()
    exchange.restored_unverified = False
    exchange.supply = pick_supply('epoch')
    states = []
    exchange.publisher = SimpleNamespace(publish=states.append)
    assert exchange.supply.expose_arrival(12, 'A')
    for observed in (True, False, True):
        exchange.at_pick = observed
        exchange.publish_supply()
        assert states[-1].valid == observed
        assert states[-1].slot_ids == ['cell/belt/pick']


def test_sidecar_binds_observed_arrivals_beyond_the_old_teaching_range(tmp_path):
    from dataclasses import asdict
    from threading import Lock
    from time import monotonic
    from types import SimpleNamespace
    from conveyor_pnp_cell.equipment_io import Sensors, Commands
    from conveyor_pnp_cell.sidecar import Exchange

    exchange = Exchange.__new__(Exchange)
    exchange.lock = Lock()
    exchange.supply = pick_supply('new-epoch')
    exchange.restored_unverified = False
    from conveyor_pnp_cell.production_ledger import ProductionLedger
    exchange.ledger = ProductionLedger(tmp_path/'pick-history.sqlite', 'test')
    exchange.pick_configured = False
    samples = []
    exchange.io = SimpleNamespace(publish=lambda **sample: samples.append(sample))
    command = Commands(65537, 22, False, False, True, True, False, 0, 0, 0, 0)
    packet = dict(sensors=asdict(Sensors(pick=True, stopped=True, extended=True)),
                  material_type_id='A', observed=65536, valid=True, acquired=monotonic(), feeder={})
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
    assert lease.success and lease.slot_id == 'cell/belt/pick' and lease.material_instance_id == 'test/65537'
    request.command = request.COMPLETE
    request.slot_id, request.reservation_id = lease.slot_id, lease.reservation_id
    request.material_type_id, request.material_instance_id = lease.material_type_id, lease.material_instance_id
    request.material_placed = True
    assert exchange.reserve(request, ConfiguredSupply.Response()).success
    exchange.sample(packet, command)
    assert samples[-1]['bound_cycle'] == 0
    assert exchange.ledger.history('test/65537')['reservations'][0]['status'] == 'RELEASED'
    exchange.ledger.close()


def test_first_piece_after_plc_reset_is_distinct_from_a_new_reservation_epoch():
    def occupant(epoch, stream):
        supply = pick_supply(epoch, stream)
        assert supply.expose_arrival(1, 'A')
        assert not supply.expose_arrival(1, 'B')
        return supply.state(valid=True).material_instance_id

    assert occupant('epoch-1', 'line-run-1') == occupant('epoch-2', 'line-run-1')
    assert occupant('epoch-2', 'line-run-2') != occupant('epoch-2', 'line-run-1')
