"""ROS sidecar inventory for the Conveyor's one physical pick position."""
import threading
from uuid import uuid4

from arm_cell_interfaces.msg import ConfiguredSupplyState
from arm_cell_interfaces.srv import ConfiguredSupply


class ConveyorPickSupply:
    def __init__(self, epoch, *, supply_id='conveyor_pick', material_id='ConveyorPart'):
        if not epoch:
            raise ValueError('A reconciled supply epoch is required')
        self.epoch, self.supply_id, self.material_id = epoch, supply_id, material_id
        self.lock = threading.Lock()
        self.cycle = 0
        self.available = False
        self.reservation = ''

    def expose_slot(self, slot):
        """Called after the equipment owner confirms this arrival at the fixed pick position."""
        number = slot.removeprefix('slot_')
        if not number.isascii() or not number.isdecimal() or slot != f'slot_{int(number)}':
            return False
        cycle = int(number)
        with self.lock:
            if cycle > self.cycle and not self.reservation and not self.available:
                self.cycle, self.available = cycle, True
            return self.cycle == cycle and self.available and not self.reservation

    @property
    def slot(self):
        return f'slot_{self.cycle}' if self.cycle else ''

    def state(self, valid):
        with self.lock:
            return ConfiguredSupplyState(
                valid=valid, supply_id=self.supply_id, material_id=self.material_id,
                supply_epoch=self.epoch, slot_ids=[self.slot] if self.cycle else [],
                occupied=[self.available] if self.cycle else [], reservation_id=self.reservation,
                selected_slot_id=self.slot if self.reservation else '', exhausted=not self.available)

    def handle(self, request, response, *, valid):
        with self.lock:
            response.supply_epoch = self.epoch
            if (request.supply_id, request.supply_epoch) != (self.supply_id, self.epoch):
                response.diagnostic_detail = 'wrong supply or epoch'
            elif request.command == request.RESERVE:
                if valid and not self.reservation:
                    if self.available:
                        self.reservation = uuid4().hex
                        response.slot_id, response.reservation_id = self.slot, self.reservation
                    response.exhausted = not self.available
                    response.success = True
            elif (self.reservation and request.reservation_id == self.reservation
                  and request.slot_id == self.slot):
                if request.command == request.ABANDON or (request.command == request.COMPLETE
                                                         and request.material_placed):
                    # A known release consumes this arrival even on a failed sequence.
                    self.available = not request.material_placed
                    self.reservation = ''
                    response.success = True
            return response
