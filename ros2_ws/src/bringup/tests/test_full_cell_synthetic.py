"""Schema checks for the deterministic synthetic full-cell scenarios."""

from pathlib import Path

import pytest
import yaml


FIXTURE_PATH = (
    Path(__file__).with_name("fixtures") / "full_cell_synthetic_scenarios.yaml"
)


def _fixture():
    document = yaml.safe_load(FIXTURE_PATH.read_text())
    return document


def test_fixture_labels_its_evidence_as_synthetic_only():
    fixture = _fixture()

    assert fixture["verification_scope"] == "synthetic_full_cell"
    assert all(
        not any(requirement.startswith("VR-SIM-") for requirement in scenario["vr"])
        for scenario in fixture["scenarios"]
    )
