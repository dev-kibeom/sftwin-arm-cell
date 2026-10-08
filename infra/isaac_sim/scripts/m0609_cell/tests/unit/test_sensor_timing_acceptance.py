import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "acceptance" / "sensor_timing_acceptance.py"
SPEC = importlib.util.spec_from_file_location("sensor_timing_acceptance", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def collector(**kwargs):
    return MODULE.SensorTimingCollector(**kwargs)


def start_request(observed, start_s=10.0):
    observed.observe_diagnostic(1, "RawPart", "", "", start_s)


def finish_request(observed, *, result_code, detail, receipt_s):
    observed.observe_diagnostic(
        0, "RawPart", "", detail, receipt_s, result_code=result_code
    )


def test_measurement_contains_only_samples_between_detecting_and_terminal_idle():
    observed = collector(sync_slop_ms=10)
    assert not observed.observe_image("rgb", 1_000_000_000, 9.9, 640, 480, "camera")

    start_request(observed)
    assert observed.observe_image("rgb", 2_000_000_000, 10.02, 640, 480, "camera")
    assert observed.observe_image("depth", 2_000_000_000, 10.04, 640, 480, "camera")
    finish_request(
        observed,
        result_code=0,
        detail="observation_ready; processing_elapsed_ms=20.0",
        receipt_s=10.15,
    )

    assert observed.complete
    assert not observed.observe_image("rgb", 3_000_000_000, 10.16, 640, 480, "camera")
    summary = observed.summary(0.3)
    assert summary["rgb"]["count"] == 1
    assert summary["depth"]["count"] == 1
    assert summary["synchronized_pairs"]["count"] == 1
    assert summary["terminal"]["result"] == "success"
    assert summary["terminal"]["diagnostic_detail"].startswith("observation_ready")
    assert summary["terminal"]["request_total_elapsed_ms"] == 150.0


def test_pairs_only_source_stamps_within_configured_sync_slop():
    observed = collector(sync_slop_ms=10)
    start_request(observed)
    observed.observe_image("rgb", 1_000_000_000, 10.02, 640, 480, "camera")
    observed.observe_image("depth", 1_009_000_000, 10.04, 640, 480, "camera")
    observed.observe_image("rgb", 2_000_000_000, 10.10, 640, 480, "camera")
    observed.observe_image("depth", 2_011_000_000, 10.12, 640, 480, "camera")

    summary = observed.summary(0.2)
    assert summary["synchronized_pairs"]["count"] == 1
    assert summary["synchronized_pairs"]["deltas_ms"] == [9.0]
    assert summary["rgb_depth_candidates"]["nearest_delta_ms"] == 9.0
    assert summary["rgb_depth_candidates"]["within_sync_slop_count"] == 1
    assert summary["rgb"]["receipt_gaps_s"] == [0.08]


def test_timeout_reports_when_no_request_pair_met_sync_slop():
    observed = collector(sync_slop_ms=10)
    start_request(observed)
    observed.observe_image("rgb", 10_000_000_000, 10.02, 640, 480, "camera")
    observed.observe_image("depth", 9_983_333_333, 10.03, 640, 480, "camera")
    observed.observe_image("rgb", 10_050_000_000, 10.20, 640, 480, "camera")
    detail = (
        "RGB/depth observed but no pair within sync tolerance; "
        "latest_rgb_depth_delta_ms=16.667; processing_elapsed_ms=not_started"
    )
    finish_request(observed, result_code=2, detail=detail, receipt_s=10.5)

    summary = observed.summary(0.6)
    assert summary["terminal"]["result"] == "timeout"
    assert summary["terminal"]["diagnostic_detail"] == detail
    assert summary["terminal"]["request_total_elapsed_ms"] == 500.0
    assert summary["rgb_depth_candidates"]["nearest_delta_ms"] == 16.666667
    assert summary["rgb_depth_candidates"]["within_sync_slop_count"] == 0
    assert (
        summary["rgb_depth_candidates"]["timeout_has_candidate_within_sync_slop"]
        is False
    )
    assert summary["synchronized_pairs"]["count"] == 0


def test_diagnostic_progress_records_first_usable_observation_time():
    observed = collector()
    start_request(observed, start_s=2.0)
    observed.observe_diagnostic(1, "RawPart", "camera", "observation_ready", 2.125)
    finish_request(observed, result_code=0, detail="ok", receipt_s=2.2)

    summary = observed.summary(0.3)
    assert (
        summary["request_elapsed_to_first_receipt_ms"][
            "first_usable_observation_diagnostic"
        ]
        == 125.0
    )
    assert MODULE.acceptance_exit_code(summary) == 0


def test_no_detect_request_is_not_reported_as_a_captured_measurement():
    observed = collector()
    observed.observe_image("rgb", 1_000_000_000, 0.0, 640, 480, "camera")
    summary = observed.summary(1.0)

    assert summary["status"] == "NO_TERMINAL_REQUEST"
    assert summary["request_lifecycle"]["captured"] is False
    assert MODULE.acceptance_exit_code(summary) == 1
