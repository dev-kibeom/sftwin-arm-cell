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
    requested: int | None = None
    reply_id: int | None = None
    reply_state: int | None = None
    awaiting_receipt: bool = False


class ConveyorIO:
    def __init__(self, client, unit=1):
        self.client = client
        self.unit = unit
        self.sequence = 0

    def _command_image(self, count):
        reply = self.client.read_input_registers(0, count, unit=self.unit)
        if reply.isError() or reply.function_code != 4 or len(reply.registers) != count:
            raise OSError(f'Invalid conveyor command block: {reply}')
        return reply.registers

    def commands(self):
        return self._decode_commands(self._command_image(30))

    def commands_with_epoch(self):
        from arm_cell_plc_adapter.conveyor_stream import decode_epoch
        image = self._command_image(32)
        return self._decode_commands(image), decode_epoch(image[30:32]) or 0

    @staticmethod
    def _decode_commands(image):
        w = image[16:30]
        if w[13] != 0xC051 or any(flag not in (0, 1) for flag in w[3:8]):
            raise ValueError('Expected the Conveyor PnP PLC application')
        trace = {}
        if image[2] == 0xD105:
            trace = dict(requested=image[0] << 16 | image[1], reply_id=image[4] << 16 | image[5],
                         reply_state=image[6], awaiting_receipt=bool(image[3]))
        return Commands(w[0] << 16 | w[1], w[2], *map(bool, w[3:8]),
                        w[8] << 16 | w[9], *w[10:13], **trace)

    def test_epoch(self, *, require_offline_reset=False):
        reply = self.client.read_input_registers(30, 2, unit=self.unit)
        if reply.isError() or reply.function_code != 4 or len(reply.registers) != 2:
            raise OSError('Cannot read Conveyor Initialize capability')
        if require_offline_reset and reply.registers[0] != 0x1A19:
            raise ValueError('CODESYS에서 최신 Conveyor.fbsdev를 다운로드하고 실행하세요 (Initialize 0x1A19).')
        from arm_cell_plc_adapter.conveyor_stream import decode_epoch
        return decode_epoch(reply.registers)

    def verify_initialized(self, epoch):
        reply = self.client.read_input_registers(0, 32, unit=self.unit)
        if reply.isError() or reply.function_code != 4 or len(reply.registers) != 32:
            raise OSError('Cannot verify Conveyor origin')
        words = reply.registers
        if words[29:] != [0xC051, 0x1A19, epoch] or any(words[:2]) or any(words[3:7]) or any(words[16:29]):
            raise ValueError('PLC acknowledged Initialize but request, equipment or result is not at origin')

    def request_initialize(self, epoch):
        reply = self.client.write_registers(26, [epoch], unit=self.unit)
        if reply.isError() or reply.function_code != 16 or reply.address != 26 or reply.count != 1:
            raise OSError('Conveyor Initialize write was not acknowledged')

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
