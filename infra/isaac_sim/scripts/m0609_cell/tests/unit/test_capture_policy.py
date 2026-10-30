import math
from dataclasses import replace

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
    expected_orientation_error_deg=None,
    rawpart_orientation_error_deg=None,
):
    return CaptureCandidate(
        prim_path=path,
        eligible=eligible,
        tcp_local_position_m=local_position,
        position_error_m=position_error_m,
        orientation_error_deg=orientation_error_deg,
        measured_opening_mm=measured_opening_mm,
        expected_orientation_error_deg=expected_orientation_error_deg,
        rawpart_orientation_error_deg=rawpart_orientation_error_deg,
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


def test_capture_eligibility_uses_volume_not_reference_or_width_errors():
    result = evaluate(
        [
            candidate(
                position_error_m=math.inf,
                orientation_error_deg=math.inf,
                measured_opening_mm=100.0,
                expected_orientation_error_deg=179.0,
                rawpart_orientation_error_deg=179.0,
            )
        ]
    )
    assert result.decision is CaptureDecision.CAPTURE_ELIGIBLE


def test_capture_eligibility_ignores_nonfinite_reference_diagnostics():
    result = evaluate(
        [candidate(position_error_m=math.nan, orientation_error_deg=math.inf)]
    )
    assert result.decision is CaptureDecision.CAPTURE_ELIGIBLE


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


def test_invalid_orientation_of_candidate_outside_capture_volume_does_not_poison_capture():
    result = evaluate(
        [
            candidate(
                "/World/OldPart",
                local_position=(0.20, 0.0, 0.0),
                position_error_m=math.nan,
                orientation_error_deg=math.inf,
            ),
            candidate("/World/CurrentPart"),
        ]
    )

    assert result.decision is CaptureDecision.CAPTURE_ELIGIBLE
    assert result.candidate.prim_path == "/World/CurrentPart"


def test_invalid_orientation_of_candidate_inside_capture_volume_is_diagnostic_only():
    result = evaluate([candidate(orientation_error_deg=math.inf)])

    assert result.decision is CaptureDecision.CAPTURE_ELIGIBLE


@pytest.mark.parametrize(
    "overrides",
    [
        {"position_error_m": 0.5},
        {"orientation_error_deg": 180.0},
        {"measured_opening_mm": 100.0},
        {"expected_orientation_error_deg": 180.0},
        {"rawpart_orientation_error_deg": 180.0},
    ],
)
def test_reference_orientation_and_opening_diagnostics_never_gate_capture(overrides):
    result = evaluate([candidate(**overrides)])
    assert result.decision is CaptureDecision.CAPTURE_ELIGIBLE


@pytest.mark.parametrize(
    "field, value",
    [
        ("capture_volume_dimensions_m", None),
        ("capture_volume_dimensions_m", (0.1, 0.0, 0.1)),
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


def test_capture_policy_returns_the_actual_candidate_assessments():
    accepted = candidate("/World/Accepted")
    outside = replace(
        accepted,
        prim_path="/World/Outside",
        tcp_local_position_m=(0.06, 0.0, 0.0),
    )
    result = CapturePolicy.evaluate(
        close_active=True,
        candidates=[accepted, outside],
        config=config(),
        runtime_available=True,
    )

    assert result.candidate is accepted
    assert [
        (item.prim_path, item.accepted, item.rejection_reason)
        for item in result.assessments
    ] == [
        ("/World/Accepted", True, None),
        ("/World/Outside", False, "outside_capture_volume"),
    ]
    outside_assessment = result.assessments[1]
    assert outside_assessment.contact_width_error_mm is None
    assert outside_assessment.contact_width_check_evaluated is False


def test_capture_policy_preserves_outside_volume_short_circuit_for_nonfinite_metrics():
    outside = replace(
        candidate("/World/Outside"),
        tcp_local_position_m=(0.06, 0.0, 0.0),
        orientation_error_deg=float("nan"),
    )
    result = CapturePolicy.evaluate(
        close_active=True,
        candidates=[outside],
        config=config(),
        runtime_available=True,
    )

    assert result.decision.value == "no_capture"
    assert result.assessments[0].rejection_reason == "outside_capture_volume"
    assert result.assessments[0].orientation_check_evaluated is False
