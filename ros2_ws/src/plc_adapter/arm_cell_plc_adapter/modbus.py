"""Cyclic Modbus TCP register exchange using the deployment's process image."""
import math

from pymodbus.client.sync import ModbusTcpClient

from .conveyor_stream import decode_epoch


REQUEST_FIELDS = ['id_high', 'id_low']
REPLY_FIELDS = ['id_high', 'id_low', 'state', 'reason', 'placement', 'heartbeat']


class ModbusRegisters:
    def __init__(self, host, port, unit, timeout, request_offset, reply_offset,
                 request_fields, reply_fields, admission_offset=-1):
        if not host or not (0 < port <= 65535 and 0 <= unit <= 255):
            raise ValueError('invalid Modbus endpoint')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout_seconds must be positive and finite')
        if (sorted(request_fields) != sorted(REQUEST_FIELDS) or
                sorted(reply_fields) != sorted(REPLY_FIELDS)):
            raise ValueError('each register block must map every field exactly once')
        if not (0 <= request_offset <= 65536 - len(request_fields) and
                0 <= reply_offset <= 65536 - len(reply_fields)):
            raise ValueError('Modbus block exceeds register range')

        if admission_offset != -1 and (not 0 <= admission_offset <= 65534 or
                not (admission_offset + 2 <= reply_offset or reply_offset + len(reply_fields) <= admission_offset)):
            raise ValueError('Mission admission requires a separate two-word holding block')
        self.admission_offset = admission_offset
        self.client = ModbusTcpClient(host, port=port, timeout=timeout)
        self.unit = unit
        self.request_offset = request_offset
        self.reply_offset = reply_offset
        self.request_fields = request_fields
        self.reply_fields = reply_fields
        self.heartbeat = 0

    def close(self):
        self.client.close()

    def read_command(self):
        count = len(self.request_fields)
        request = self.client.read_input_registers(self.request_offset, count, unit=self.unit)
        if request.isError() or request.function_code != 4 or len(request.registers) != count:
            raise OSError(f'invalid Modbus request block: {request}')
        words = dict(zip(self.request_fields, request.registers))
        return (words['id_high'] << 16) | words['id_low']

    def read_conveyor_command(self):
        # One FC04 image binds the robot request to the PLC test that issued it.
        reply = self.client.read_input_registers(0, 32, unit=self.unit)
        if (reply.isError() or reply.function_code != 4 or len(reply.registers) != 32
                or reply.registers[29] != 0xC051):
            raise OSError('Invalid Conveyor PLC command image')
        try:
            epoch = decode_epoch(reply.registers[30:32]) or 0
        except ValueError as error:
            raise OSError(str(error)) from error
        return reply.registers[0] << 16 | reply.registers[1], epoch

    def write_receipt(self, receipt, *, conveyor_feedback=()):
        self.heartbeat = (self.heartbeat + 1) & 0xffff
        fields = dict(id_high=receipt.command_id >> 16, id_low=receipt.command_id & 0xffff,
                      state=receipt.state, reason=receipt.reason, placement=receipt.placement,
                      heartbeat=self.heartbeat)
        words = [fields[name] for name in self.reply_fields]
        self.write_words(self.reply_offset, words + list(conveyor_feedback))

    def write_admission(self, sequence, available):
        self.write_words(self.admission_offset, [sequence, int(available)])

    def write_words(self, offset, words):
        ack = self.client.write_registers(offset, words, unit=self.unit)
        if (ack.isError() or ack.function_code != 16 or
                ack.address != offset or ack.count != len(words)):
            raise OSError(f'invalid Modbus reply acknowledgement: {ack}')
