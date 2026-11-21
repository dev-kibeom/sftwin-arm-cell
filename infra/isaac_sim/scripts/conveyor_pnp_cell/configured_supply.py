"""ROS sidecar inventory for the Conveyor's one physical pick position."""
import threading
from uuid import uuid4

from arm_cell_interfaces.msg import ConfiguredSupplyState
from .supply import material_instance_id


class ConveyorPickSupply:
    def __init__(self, epoch, *, supply_id, material_id, slot_id, material_type_ids, stream_id):
        if not epoch:
            raise ValueError('A reconciled supply epoch is required')
        self.epoch, self.supply_id, self.material_id = epoch, supply_id, material_id
        self.stream_id = stream_id
        self.slot = slot_id
        self.material_type_ids = frozenset(material_type_ids)
        self.material_type_id = ''
        self.lock = threading.Lock()
        self.cycle = 0
        self.available = False
        self.reservation = ''

    @property
    def material_instance_id(self):
        return material_instance_id(self.stream_id, self.cycle) if self.available else ''

    def expose_arrival(self, cycle, material_type_id):
        """Bind the feed receipt to the observed occupant of the pick location."""
        if type(cycle) is not int or cycle <= 0 or material_type_id not in self.material_type_ids:
            return False
        with self.lock:
            if cycle > self.cycle and not self.available:
                self.cycle, self.material_type_id, self.available = cycle, material_type_id, True
            return (self.available and not self.reservation and self.cycle == cycle
                    and self.material_type_id == material_type_id)

    def state(self, valid):
        with self.lock:
            return ConfiguredSupplyState(
                valid=valid, supply_id=self.supply_id, material_id=self.material_id,
                supply_epoch=self.epoch, slot_ids=[self.slot],
                material_type_id=self.material_type_id if self.available else '',
                material_instance_id=self.material_instance_id,
                occupied=[self.available], reservation_id=self.reservation,
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
                        response.material_instance_id = self.material_instance_id
                        response.material_type_id = self.material_type_id
                        response.slot_id, response.reservation_id = self.slot, self.reservation
                    response.exhausted = not self.available
                    response.success = True
            elif (self.reservation and request.reservation_id == self.reservation
                  and request.slot_id == self.slot and request.material_type_id == self.material_type_id
                  and request.material_instance_id == self.material_instance_id):
                if request.command == request.ABANDON or (request.command == request.COMPLETE
                                                         and request.material_placed):
                    # A known release consumes this arrival even on a failed sequence.
                    self.available = not request.material_placed
                    self.reservation = ''
                    response.success = True
            return response
