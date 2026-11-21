"""Conveyor equipment callbacks, called serially by the Cell update loop."""


def material_instance_id(stream_id, cycle):
    """PLC material numbers are unique within one reconciled production stream."""
    return f'{stream_id}/{cycle}'


class ConveyorFeeder:
    def __init__(self, material_id, *, last_generated, material_type_ids=None):
        # Restore only after reconciling the last physical creation with the PLC.
        self.material_id = material_id
        self.material_type_ids = tuple(material_type_ids or (material_id,))
        self.material_type_id = self.material_type_ids[0]
        self.last_generated = last_generated
        self.pending = None
        self.remaining = 0
        self.automatic = False

    def bind_ui(self, *, set_automatic=None):
        from isaac_runtime.cell_supply import register_supply_port
        register_supply_port(self, dict(materials=[dict(id=t) for t in self.material_type_ids],
                             count_label='Available parts', default_count=1),
                             self.configure, self.status, set_automatic=set_automatic or self.set_automatic)

    def configure(self, material, count):
        if (self.automatic or self.pending is not None or material not in self.material_type_ids
                or type(count) is not int or not 1 <= count <= 65535):
            return dict(accepted=False, detail='Pause automatic feed and reconcile pending stock.')
        self.material_type_id = material
        self.remaining = count
        return dict(accepted=True, detail=f'{count} parts available')

    def set_automatic(self, enabled):
        self.automatic = enabled
        return dict(accepted=True, detail='Automatic feed enabled' if enabled else 'Automatic feed paused')

    def status(self):
        mode = 'Automatic' if self.automatic else 'Paused'
        detail = f'{mode} · {self.remaining} available · last generated {self.last_generated}'
        if self.pending is not None:
            detail += f' · Awaiting creation confirmation for {self.pending}'
        return dict(available=True, automatic=self.automatic, queued=self.remaining, detail=detail)

    def feed(self, cycle, spawn_one):
        """A positive creation receipt is independent of the infeed arrival sensor."""
        if (not self.automatic or not self.remaining or self.pending is not None
                or cycle != self.last_generated + 1):
            return
        self.pending = cycle
        if spawn_one(self.material_type_id, cycle) is True:
            self.last_generated = cycle
            self.remaining -= 1
            self.pending = None


def bind_confirmed_pick(command, observed_cycle, material_type_id, expose_arrival):
    """Acknowledge the PLC material number only for the confirmed, reservable arrival."""
    if not command.confirmed or not observed_cycle or command.cycle != observed_cycle:
        return 0
    return observed_cycle if expose_arrival(observed_cycle, material_type_id) is True else 0
