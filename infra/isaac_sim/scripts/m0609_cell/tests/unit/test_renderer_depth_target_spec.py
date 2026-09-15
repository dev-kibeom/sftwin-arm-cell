from dataclasses import FrozenInstanceError

import numpy as np
import pytest
import json

from camera_tooling.renderer_depth_target_spec import (
    CANONICAL_FRAME,
    CANDIDATE_A,
    TARGET_DEPTHS_M,
    fallback_candidates,
)
from renderer_depth_fixture_suitability_probe import (
    SuitabilityCriteria,
    evaluate_candidate,
    evaluate_candidates,
    project_target,
    serialize_report,
)


class CameraInfo:
    width = 1280
    height = 720
    k = [1465.9986, 0.0, 640.0, 0.0, 1465.9986, 360.0, 0.0, 0.0, 1.0]


def valid_depth():
    return np.full((CameraInfo.height, CameraInfo.width), 2.0, dtype=np.float32)


def test_canonical_spec_has_three_stable_immutable_targets_and_one_z_source():
    assert [target.target_id for target in CANDIDATE_A.targets] == [
        "z080",
        "z110",
        "z140",
    ]
    assert len({target.target_id for target in CANDIDATE_A.targets}) == 3
    assert tuple(target.front_optical_xyz_m[2] for target in CANDIDATE_A.targets) == (
        0.8,
        1.1,
        1.4,
    )
    assert tuple(target.front_optical_xyz_m[:2] for target in CANDIDATE_A.targets) == (
        (-0.2739429677526156, -0.12114609330892565),
        (0.3196456043049693, -0.18158271418263516),
        (0.5090045879905422, 0.0),
    )
    assert TARGET_DEPTHS_M == (0.8, 1.1, 1.4)
    assert all(target.frame_id == CANONICAL_FRAME for target in CANDIDATE_A.targets)
    assert all(target.dimensions_m == (0.08, 0.08, 0.004) for target in CANDIDATE_A.targets)
    assert [target.material_id for target in CANDIDATE_A.targets] == [
        "renderer_depth_red",
        "renderer_depth_green",
        "renderer_depth_blue",
    ]
    with pytest.raises(FrozenInstanceError):
        CANDIDATE_A.targets[0].target_id = "changed"


def test_canonical_spec_contains_only_fixture_fields():
    target = CANDIDATE_A.targets[0]
    assert set(target.__dataclass_fields__) == {
        "target_id",
        "frame_id",
        "front_optical_xyz_m",
        "dimensions_m",
        "rgb",
        "material_id",
    }


def test_fallback_candidates_are_finite_deterministic_and_camera_derived():
    first = fallback_candidates(CameraInfo)
    second = fallback_candidates(CameraInfo)
    assert [candidate.candidate_id for candidate in first] == ["B", "C"]
    assert first == second
    assert all(len(candidate.targets) == 3 for candidate in first)
    assert all(
        target.front_optical_xyz_m[2] in TARGET_DEPTHS_M
        for candidate in first
        for target in candidate.targets
    )
    assert first[0].targets[0].front_optical_xyz_m != CANDIDATE_A.targets[0].front_optical_xyz_m


def test_candidate_a_is_evaluated_first_and_fallbacks_stop_when_it_passes(monkeypatch):
    import renderer_depth_fixture_suitability_probe as probe

    monkeypatch.setattr(
        probe,
        "fallback_candidates",
        lambda _: pytest.fail("fallback layouts must not be created after Candidate A passes"),
    )
    report = evaluate_candidates(valid_depth(), CameraInfo)
    assert [item["candidate_id"] for item in report["evaluated_candidates"]] == ["A"]
    assert report["final_recommendation"] == "A"
    assert report["fallback_candidates"] == [
        {"candidate_id": "B", "status": "NOT EVALUATED"},
        {"candidate_id": "C", "status": "NOT EVALUATED"},
    ]


def test_finite_fallbacks_are_evaluated_only_after_candidate_a_fails():
    report = evaluate_candidates(
        np.zeros((CameraInfo.height, CameraInfo.width), dtype=np.float32), CameraInfo
    )
    assert [item["candidate_id"] for item in report["evaluated_candidates"]] == [
        "A",
        "B",
        "C",
    ]
    assert report["final_recommendation"] is None
    assert report["status"] == "FAIL"
    assert report["fallback_candidates"] == []


def test_fallback_b_is_recommended_and_later_candidates_are_not_evaluated():
    depth = np.full((CameraInfo.height, CameraInfo.width), 0.1, dtype=np.float32)
    candidate_b = fallback_candidates(CameraInfo)[0]
    for target in candidate_b.targets:
        u, v = project_target(target, CameraInfo)
        depth[round(v), round(u)] = 2.0
    report = evaluate_candidates(depth, CameraInfo)
    assert [item["candidate_id"] for item in report["evaluated_candidates"]] == [
        "A",
        "B",
    ]
    assert report["final_recommendation"] == "B"
    assert report["fallback_candidates"] == [
        {"candidate_id": "C", "status": "NOT EVALUATED"}
    ]


def test_synthetic_candidate_meeting_fixed_suitability_criteria_passes():
    result = evaluate_candidate(
        CANDIDATE_A, valid_depth(), CameraInfo, SuitabilityCriteria()
    )
    assert result["status"] == "PASS"
    assert all(target["status"] == "PASS" for target in result["targets"])
    assert all(target["border_pass"] for target in result["targets"])
    assert all(target["depth_clearance_pass"] for target in result["targets"])
    assert result["separation_pass"]


@pytest.mark.parametrize(
    "depth,info,expected_reason",
    [
        (
            np.full((80, 100), 2.0),
            type(
                "Narrow",
                (),
                {"width": 100, "height": 80, "k": CameraInfo.k},
            ),
            "outside image",
        ),
        (
            np.full((720, 1280), 2.0),
            type(
                "Border",
                (),
                {
                    "width": 1280,
                    "height": 720,
                    "k": [333.0, 0, 125.0, 0, 333.0, 100.0, 0, 0, 1],
                },
            ),
            "border",
        ),
    ],
)
def test_projection_outside_image_or_with_insufficient_border_is_rejected(
    depth, info, expected_reason
):
    result = evaluate_candidate(CANDIDATE_A, depth, info, SuitabilityCriteria())
    assert result["status"] == "FAIL"
    assert expected_reason in " ".join(
        str(target["reasons"]) for target in result["targets"]
    )


def test_insufficient_inter_target_separation_is_rejected():
    info = type(
        "NarrowFov",
        (),
        {"width": 1280, "height": 720, "k": [100.0, 0, 640, 0, 100.0, 360, 0, 0, 1]},
    )
    result = evaluate_candidate(CANDIDATE_A, valid_depth(), info, SuitabilityCriteria())
    assert not result["separation_pass"]
    assert result["status"] == "FAIL"


def test_insufficient_baseline_depth_clearance_is_rejected():
    depth = np.full((720, 1280), 1.44, dtype=np.float32)
    result = evaluate_candidate(CANDIDATE_A, depth, CameraInfo, SuitabilityCriteria())
    assert result["status"] == "FAIL"
    assert any(not target["depth_clearance_pass"] for target in result["targets"])


def test_optional_diagnostic_dump_does_not_change_probe_decision(tmp_path):
    from renderer_depth_fixture_suitability_probe import write_diagnostic_dump

    report = evaluate_candidates(valid_depth(), CameraInfo)
    before = dict(report)
    output = tmp_path / "diagnostic.json"
    write_diagnostic_dump(report, output)
    assert output.is_file()
    assert report == before


def test_probe_report_serializes_numpy_boolean_values():
    info = type(
        "NumpyCameraInfo",
        (),
        {
            "width": CameraInfo.width,
            "height": CameraInfo.height,
            "k": np.asarray(CameraInfo.k, dtype=np.float64),
        },
    )
    report = evaluate_candidates(valid_depth(), info)
    decoded = json.loads(serialize_report(report))
    assert decoded["evaluated_candidates"][0]["targets"][0]["border_pass"] is True
    assert decoded["evaluated_candidates"][0]["targets"][0]["depth_clearance_pass"] is True


def test_probe_has_no_local_artifact_input_dependency():
    report = evaluate_candidates(valid_depth(), CameraInfo)
    assert report["status"] == "PASS"
    assert "input_path" not in report
