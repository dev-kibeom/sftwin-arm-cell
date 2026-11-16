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
