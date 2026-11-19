import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tape_roll_cell.supply import TapeSupply


def test_failure_preserves_slot_and_completion_requires_transfer_and_release():
    supply = TapeSupply(3)
    assert supply.select() == 0
    for transferred, released in [(False, False), (True, False), (False, True)]:
        with pytest.raises(ValueError):
            supply.complete(0, transferred=transferred, released=released)
        assert supply.snapshot() == {'occupied': [True]*3, 'completed': 0, 'selected': 0, 'exhausted': False}
    supply.fail()
    assert supply.select() == 0
    supply.complete(0, transferred=True, released=True)
    with pytest.raises(ValueError):
        supply.complete(0, transferred=True, released=True)
    assert supply.select() == 1


def test_exhaustion_and_stop_restore_initial_supply():
    supply = TapeSupply(3)
    for slot in range(3):
        assert supply.select() == slot
        supply.complete(slot, transferred=True, released=True)
    assert supply.select() is None
    assert supply.snapshot()['completed'] == 0
    assert supply.snapshot()['exhausted']
    supply.reset()
    assert supply.snapshot() == {'occupied': [True]*3, 'completed': 0, 'selected': None, 'exhausted': False}
