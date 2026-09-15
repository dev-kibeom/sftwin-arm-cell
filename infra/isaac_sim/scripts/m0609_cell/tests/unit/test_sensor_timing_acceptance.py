import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "acceptance" / "sensor_timing_acceptance.py"
SPEC = importlib.util.spec_from_file_location("sensor_timing_acceptance", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def collector(**kwargs):
    return MODULE.ExactTripleCollector(**kwargs)


def test_exact_triple_succeeds():
    observed = collector(required_samples=1)
    assert not observed.observe("rgb", 10)
    assert not observed.observe("depth", 10)
    assert observed.observe("camera_info", 10)
    assert observed.triples[0]["spread_ns"] == 0


def test_camera_info_mismatch_is_not_accepted():
    observed = collector(required_samples=1)
    observed.observe("rgb", 10)
    observed.observe("depth", 10)
    observed.observe("camera_info", 11)
    assert not observed.complete


def test_one_nanosecond_mismatch_is_not_accepted():
    observed = collector(required_samples=1)
    observed.observe("rgb", 10)
    observed.observe("depth", 11)
    observed.observe("camera_info", 10)
    assert not observed.complete


def test_late_camera_info_and_arbitrary_order_succeed():
    observed = collector(required_samples=1)
    observed.observe("depth", 20)
    observed.observe("rgb", 20)
    assert not observed.complete
    assert observed.observe("camera_info", 20)
    assert observed.complete


def test_missing_stream_remains_unverified_until_timeout_boundary():
    observed = collector(required_samples=1)
    observed.observe("rgb", 30)
    observed.observe("depth", 30)
    summary = observed.summary("NOT VERIFIED", 1.0)
    assert MODULE.acceptance_exit_code(summary) == 1
    assert summary["observed_samples"] == 0


def test_sample_count_is_bounded_and_duplicate_is_deterministic():
    observed = collector(required_samples=2)
    for stamp in (1, 2, 3):
        observed.observe("rgb", stamp)
        observed.observe("depth", stamp)
        observed.observe("camera_info", stamp)
    assert len(observed.triples) == 2
    assert observed.complete
    assert observed.observe("rgb", 3) is False


def test_pending_state_is_bounded_with_deterministic_oldest_drop():
    observed = collector(required_samples=1, max_pending=2)
    observed.observe("rgb", 1)
    observed.observe("rgb", 2)
    observed.observe("rgb", 3)
    assert len(observed.pending["rgb"]) == 2
    assert 1 not in observed.pending["rgb"]
    assert observed.dropped_messages == 1


def test_success_exit_code_is_zero():
    observed = collector(required_samples=1)
    for stream in MODULE.STREAMS:
        observed.observe(stream, 40)
    assert MODULE.acceptance_exit_code(observed.summary("VERIFIED", 0.1)) == 0
