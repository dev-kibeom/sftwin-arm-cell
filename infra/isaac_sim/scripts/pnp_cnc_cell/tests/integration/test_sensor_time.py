from types import SimpleNamespace
from sensor_observer import SensorObserver


def test_wall_deadline_progresses_while_sim_clock_is_paused():
    now = [100.0]
    observer = SensorObserver(2, monotonic=lambda: now[0])
    observer.receive_clock(SimpleNamespace(clock=SimpleNamespace(sec=5, nanosec=0)))
    started = now[0]
    now[0] += 1.1
    assert observer.deadline_expired(started, 1.0)
