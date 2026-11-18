"""Cell-owned cartridge inventory; failed picks never consume a slot."""


class TapeSupply:
    def __init__(self, count):
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError('Cartridge requires a positive slot count')
        self._count = count
        self.reset()

    def reset(self):
        self._occupied = [True] * self._count
        self._selected = None

    def select(self):
        if self._selected is None:
            self._selected = next((i for i, occupied in enumerate(self._occupied) if occupied), None)
        return self._selected

    def complete(self, slot, *, transferred, released):
        if slot != self._selected or slot is None or not self._occupied[slot]:
            raise ValueError('Completion must identify the active occupied slot')
        if not transferred or not released:
            raise ValueError('Completion requires observed physical transfer and release')
        self._occupied[slot] = False
        self._selected = None

    def fail(self):
        self._selected = None

    def snapshot(self):
        return dict(occupied=list(self._occupied), completed=self._occupied.count(False),
                    selected=self._selected, exhausted=not any(self._occupied))


class ConfiguredTapeSupply(TapeSupply):
    """Session-fenced reservations; physical completion is supplied by the Cell."""
    def __init__(self, count, supply_id='tape_cartridge', material_id='TapeRoll17'):
        self.supply_id, self.material_id = supply_id, material_id
        super().__init__(count)

    def reset(self):
        import uuid
        super().reset()
        self.epoch = uuid.uuid4().hex
        self.reservation_id = ''
        self._completed_token = None
        self._loaded = True

    def unload(self):
        self._loaded = False
        self._occupied = [False]*self._count
        self._selected = None
        self.reservation_id = ''

    def snapshot(self):
        state = super().snapshot()
        if not getattr(self,'_loaded',True):state.update(completed=0,exhausted=False)
        return state

    def _response(self, success, detail=''):
        snapshot = self.snapshot()
        return dict(success=success, diagnostic_detail=detail, supply_epoch=self.epoch,
                    slot_id=f'slot_{self._selected}' if self._selected is not None else '',
                    reservation_id=self.reservation_id, exhausted=snapshot['exhausted'],
                    completed_count=snapshot['completed'])

    def handle(self, request, *, valid, completion_observed):
        import uuid
        if request.get('supply_id') != self.supply_id:
            return self._response(False, 'unknown_supply')
        command = request.get('command')
        if command == 0:
            if request.get('supply_epoch') != self.epoch:
                return self._response(False, 'stale_supply_epoch')
            if self._selected is not None:
                return self._response(False, 'reservation_active')
            if not valid:
                return self._response(False, 'cell_not_ready')
            if self._selected is None:
                slot = self.select()
                if slot is None:
                    return self._response(True, 'exhausted')
                self.reservation_id = uuid.uuid4().hex
                self._completed_token = None
            return self._response(True)
        token = (request.get('supply_epoch'), request.get('reservation_id'), request.get('slot_id'))
        expected = (self.epoch, self.reservation_id,
                    f'slot_{self._selected}' if self._selected is not None else '')
        if command == 1 and token == self._completed_token and self._selected is None:
            return self._response(True, 'already_completed')
        if (self._selected is None or not self.reservation_id or token != expected):
            return self._response(False, 'stale_or_wrong_reservation')
        if command == 2:
            self.fail()
            self.reservation_id = ''
            return self._response(True, 'abandoned')
        if command != 1:
            return self._response(False, 'unknown_command')
        if not valid or request.get('material_placed') is not True or not completion_observed(self._selected):
            return self._response(False, 'physical_completion_not_confirmed')
        self.complete(self._selected, transferred=True, released=True)
        self._completed_token = token
        self.reservation_id = ''
        return self._response(True, 'completed')

    def state(self, *, valid, detail=''):
        snapshot = self.snapshot()
        return dict(supply_id=self.supply_id, material_id=self.material_id, supply_epoch=self.epoch,
                    slot_ids=[f'slot_{i}' for i in range(self._count)], occupied=snapshot['occupied'],
                    selected_slot_id=f'slot_{self._selected}' if self._selected is not None else '',
                    reservation_id=self.reservation_id, completed_count=snapshot['completed'],
                    valid=bool(valid), exhausted=snapshot['exhausted'], diagnostic_detail=detail)
