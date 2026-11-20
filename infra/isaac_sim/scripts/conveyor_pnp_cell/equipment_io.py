"""Conveyor Cell Modbus image for the equipment sidecar and commissioning fixture."""
from dataclasses import dataclass, fields


@dataclass
class Sensors:
    infeed: bool = False
    pick: bool = False
    stopped: bool = False
    extended: bool = False
    occupied: bool = False
    path_clear: bool = False
    drive_available: bool = False
    pick_zone_clear: bool = False
    discharge_zone_clear: bool = False
    permitted: bool = False
    on_belt: bool = False


@dataclass(frozen=True)
class Commands:
    cycle: int
    phase: int
    feed: bool
    run: bool
    extend: bool
    confirmed: bool
    discharge: bool
    committed: int
    result_state: int
    reason: int
    placement: int


class ConveyorIO:
    def __init__(self, client, unit=1):
        self.client = client
        self.unit = unit
        self.sequence = 0

    def commands(self):
        reply = self.client.read_input_registers(16, 14, unit=self.unit)
        if reply.isError() or reply.function_code != 4 or len(reply.registers) != 14:
            raise OSError(f'Invalid conveyor command block: {reply}')
        w = reply.registers
        if w[13] != 0xC051 or any(flag not in (0, 1) for flag in w[3:8]):
            raise ValueError('Expected the Conveyor PnP PLC application')
        return Commands(w[0] << 16 | w[1], w[2], *map(bool, w[3:8]),
                        w[8] << 16 | w[9], *w[10:13])

    def publish(self, *, valid, feeder, sensors, bound_cycle):
        # Advance only on acquisition of a complete new equipment sample.
        self.sequence = (self.sequence + 1) & 0xffff
        bits = sum(int(getattr(sensors, f.name)) << i for i, f in enumerate(fields(Sensors)))
        generated = feeder.last_generated
        words = [self.sequence, int(valid), int(feeder.automatic),
                 int(feeder.material_id == 'ConveyorPart'), feeder.remaining, bits,
                 generated >> 16, generated & 0xffff, bound_cycle >> 16, bound_cycle & 0xffff]
        ack = self.client.write_registers(16, words, unit=self.unit)
        if ack.isError() or ack.function_code != 16 or ack.address != 16 or ack.count != len(words):
            raise OSError(f'Invalid conveyor feedback acknowledgement: {ack}')
