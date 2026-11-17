"""Per-physics-step simulation contact observations for inference and diagnostics."""


class PhysicalContactObserver:
    def __init__(self):
        import omni.physx
        from omni.physx.bindings._physx import ContactEventType
        from pxr import PhysicsSchemaTools
        self._lost = ContactEventType.CONTACT_LOST
        self._path = PhysicsSchemaTools.intToSdfPath
        self._pending = []
        self._pending_steps = 0
        self.snapshot = []
        self._subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self._report)

    def _report(self, headers, data):
        self._pending_steps += 1
        for header in headers:
            self._pending.append(dict(
                actor0=str(self._path(header.actor0)), actor1=str(self._path(header.actor1)),
                collider0=str(self._path(header.collider0)) if hasattr(header, 'collider0') else None,
                collider1=str(self._path(header.collider1)) if hasattr(header, 'collider1') else None,
                lost=header.type == self._lost,
                points=[dict(position=tuple(point.position), normal=tuple(point.normal),
                             impulse=tuple(point.impulse), separation_m=float(point.separation))
                        for point in data[header.contact_data_offset:header.contact_data_offset + header.num_contact_data]]))

    def advance(self):
        # Multiple unconsumed report batches have no trustworthy single-step dt.
        self.snapshot = self._pending if self._pending_steps <= 1 else None
        self._pending = []
        self._pending_steps = 0

    def clear(self):
        self.snapshot, self._pending = [], []
        self._pending_steps = 0

    def close(self):
        self._subscription = None
        self.clear()
