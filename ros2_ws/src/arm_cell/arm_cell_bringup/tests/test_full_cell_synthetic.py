"""Schema checks for the deterministic synthetic full-cell scenarios."""

from pathlib import Path

import pytest
import yaml


FIXTURE_PATH = (
    Path(__file__).with_name("fixtures") / "full_cell_synthetic_scenarios.yaml"
)


REQUIRED_SCENARIOS = {
    "nominal_mission",
    "target_depletion",
    "vision_error",
    "motion_failure",
    "controlled_stop",
    "immediate_stop",
    "emergency_stop",
    "communication_loss",
    "premature_undock",
    "packml_abort",
    "external_clear_without_safety_reset",
    "recovery_success",
    "recovery_failure",
    "mission_cancellation",
    "capability_downgrade",
}

FORBIDDEN_RESULT_FIELDS = {
    "safety_capability_enforced",
    "direct_stop_independent_of_orchestration",
    "motion_stopped_before_recovery",
    "mission_result_preserved",
    "no_automatic_retry",
}


def _scenarios():
    document = yaml.safe_load(FIXTURE_PATH.read_text())
    return {scenario["name"]: scenario for scenario in document["scenarios"]}


def test_fixture_contains_inputs_and_outcomes_not_preasserted_invariants():
    scenarios = _scenarios()

    assert set(scenarios) == REQUIRED_SCENARIOS
    for scenario in scenarios.values():
        assert set(scenario) == {"name", "setup", "stimulus", "expected", "vr"}
        assert scenario["setup"]
        assert scenario["stimulus"]
        assert scenario["expected"]
        assert scenario["vr"]
        assert FORBIDDEN_RESULT_FIELDS.isdisjoint(scenario["expected"])


def test_fixture_does_not_claim_live_isaac_evidence():
    assert all(
        "VR-SIM-DEPTH-PIXEL" not in scenario["vr"] for scenario in _scenarios().values()
    )
