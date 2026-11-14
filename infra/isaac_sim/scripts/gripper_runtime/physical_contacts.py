"""Simulation-only contact diagnostics. These never certify HELD."""


class PhysicalContactObserver:
    def __init__(self):
        import omni.physx
        from omni.physx.bindings._physx import ContactEventType
        from pxr import PhysicsSchemaTools
        self._lost = ContactEventType.CONTACT_LOST
        self._path = PhysicsSchemaTools.intToSdfPath
        self._pending = []
        self.snapshot = []
        self._subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self._report)

    def _report(self, headers, data):
        for header in headers:
            self._pending.append(dict(
                actor0=str(self._path(header.actor0)), actor1=str(self._path(header.actor1)),
                lost=header.type == self._lost,
                points=[dict(position=tuple(point.position), normal=tuple(point.normal),
                             impulse=tuple(point.impulse))
                        for point in data[header.contact_data_offset:header.contact_data_offset + header.num_contact_data]]))

    def advance(self):
        self.snapshot, self._pending = self._pending, []

    def clear(self):
        self.snapshot, self._pending = [], []

    def close(self):
        self._subscription = None
        self.clear()
