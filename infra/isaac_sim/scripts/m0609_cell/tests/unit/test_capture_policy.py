import math

import pytest

from gripper_runtime.grasp_policy import (
    CaptureCandidate,
    CaptureConfig,
    CaptureDecision,
    CapturePolicy,
)


def config(**overrides):
    values = {
        "capture_volume_dimensions_m": (0.10, 0.10, 0.10),
        "position_tolerance_m": 0.01,
        "orientation_tolerance_deg": 5.0,
        "expected_contact_width_mm": 12.0,
        "contact_width_tolerance_mm": 1.0,
    }
    values.update(overrides)
    return CaptureConfig(**values)


def candidate(
    path="/World/Cube",
    *,
    eligible=True,
    local_position=(0.0, 0.0, 0.0),
    position_error_m=0.005,
    orientation_error_deg=2.0,
    measured_opening_mm=12.0,
):
    return CaptureCandidate(
        prim_path=path,
        eligible=eligible,
        tcp_local_position_m=local_position,
        position_error_m=position_error_m,
        orientation_error_deg=orientation_error_deg,
        measured_opening_mm=measured_opening_mm,
    )


def evaluate(candidates, *, close_active=True, capture_config=None):
    return CapturePolicy.evaluate(
        close_active=close_active,
        candidates=candidates,
        config=capture_config or config(),
        runtime_available=True,
    )


def test_no_candidate_does_not_attach():
    result = evaluate([])
    assert result.decision is CaptureDecision.NO_CAPTURE
    assert result.candidate is None


def test_exactly_one_valid_candidate_is_capture_eligible():
    result = evaluate([candidate()])
    assert result.decision is CaptureDecision.CAPTURE_ELIGIBLE
    assert result.candidate.prim_path == "/World/Cube"


def test_multiple_valid_candidates_are_ambiguous():
    result = evaluate([candidate("/World/CubeA"), candidate("/World/CubeB")])
    assert result.decision is CaptureDecision.AMBIGUOUS
    assert result.candidate is None


def test_ineligible_candidate_is_ignored():
    result = evaluate([candidate(eligible=False)])
    assert result.decision is CaptureDecision.NO_CAPTURE


def test_candidate_outside_capture_volume_is_rejected():
    result = evaluate([candidate(local_position=(0.051, 0.0, 0.0))])
    assert result.decision is CaptureDecision.NO_CAPTURE


@pytest.mark.parametrize(
    "field, value",
    [
        ("position_error_m", 0.01),
        ("orientation_error_deg", 5.0),
        ("measured_opening_mm", 13.0),
    ],
)
def test_tolerance_boundaries_are_inclusive(field, value):
    kwargs = {field: value}
    result = evaluate([candidate(**kwargs)])
    assert result.decision is CaptureDecision.CAPTURE_ELIGIBLE


@pytest.mark.parametrize(
    "field, value",
    [
        ("position_error_m", 0.0100001),
        ("orientation_error_deg", 5.0001),
        ("measured_opening_mm", 13.0001),
    ],
)
def test_values_just_outside_tolerance_are_rejected(field, value):
    kwargs = {field: value}
    result = evaluate([candidate(**kwargs)])
    assert result.decision is CaptureDecision.NO_CAPTURE


@pytest.mark.parametrize(
    "field, value",
    [
        ("capture_volume_dimensions_m", None),
        ("position_tolerance_m", 0.0),
        ("orientation_tolerance_deg", -1.0),
        ("expected_contact_width_mm", math.nan),
        ("contact_width_tolerance_mm", -0.1),
    ],
)
def test_invalid_or_missing_configuration_is_not_ready(field, value):
    result = evaluate([candidate()], capture_config=config(**{field: value}))
    assert result.decision is CaptureDecision.INVALID_CONFIG


def test_missing_configuration_is_invalid_config():
    result = CapturePolicy.evaluate(
        close_active=True,
        candidates=[candidate()],
        config=None,
        runtime_available=True,
    )
    assert result.decision is CaptureDecision.INVALID_CONFIG


def test_inactive_close_command_cannot_capture():
    result = evaluate([candidate()], close_active=False)
    assert result.decision is CaptureDecision.NO_CAPTURE


def test_unavailable_runtime_is_unknown():
    result = CapturePolicy.evaluate(
        close_active=True,
        candidates=[candidate()],
        config=config(),
        runtime_available=False,
    )
    assert result.decision is CaptureDecision.UNKNOWN_RUNTIME
