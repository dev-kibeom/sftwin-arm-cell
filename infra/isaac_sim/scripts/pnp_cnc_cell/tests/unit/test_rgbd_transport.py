from types import SimpleNamespace
import pytest
from rgbd_assertions import ExactPairer


def msg(sec, nanosec):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
    )


def test_exact_pairing_handles_reverse_arrival_and_epoch_separation():
    pairer = ExactPairer(2)
    assert pairer.add_depth(msg(2, 3)) is None
    assert pairer.add_color(msg(2, 3))
    pairer.add_color(msg(3, 0))
    assert pairer.unmatched_count == 1


def test_exact_pairing_rejects_duplicates_and_pending_overflow():
    pairer = ExactPairer(1)
    pairer.add_color(msg(1, 1))
    with pytest.raises(ValueError):
        pairer.add_color(msg(1, 1))
    with pytest.raises(ValueError):
        pairer.add_color(msg(1, 2))
