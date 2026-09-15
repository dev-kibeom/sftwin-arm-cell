import pytest

from depth_semantics import classify_depth_samples


def samples(values):
    return [
        {
            "actual_depth": actual,
            "expected_optical_z_m": z,
            "expected_range_m": distance,
        }
        for actual, z, distance in values
    ]


def test_classifies_metric_optical_axis_z_from_independent_depths():
    result = classify_depth_samples(
        samples([(0.5, 0.5, 0.51), (1.0, 1.0, 1.03), (2.0, 2.0, 2.1)]), 0.1, 10.0, 0.002
    )
    assert result["status"] == "VERIFIED"
    assert result["semantic"] == "optical_axis_z_m"
    assert result["depth_unit"] == "metres"


def test_classifies_normalized_depth_buffer_without_calling_it_metres():
    result = classify_depth_samples(
        samples(
            [(0.8080808, 0.5, 0.51), (0.9090909, 1.0, 1.03), (0.9595960, 2.0, 2.1)]
        ),
        0.1,
        10.0,
        0.002,
        normalized_tolerance=0.0001,
    )
    assert result["status"] == "VERIFIED"
    assert result["semantic"] == "perspective_depth_buffer_optical_z"
    assert result["depth_unit"] == "normalized"


def test_refuses_single_plane_samples_even_when_the_values_match():
    result = classify_depth_samples(
        samples([(1.0, 1.0, 1.0), (1.0, 1.0, 1.01), (1.0, 1.0, 1.02)]), 0.1, 10.0, 0.002
    )
    assert result["status"] == "NOT VERIFIED"
    assert "independent optical depths" in result["reason"]


def test_rejects_invalid_configuration():
    with pytest.raises(ValueError, match="clipping"):
        classify_depth_samples(samples([(1.0, 1.0, 1.0)] * 3), 10.0, 0.1, 0.002)


def test_normalized_candidate_needs_an_explicit_dimensionless_tolerance():
    result = classify_depth_samples(
        samples(
            [(0.8080808, 0.5, 0.51), (0.9090909, 1.0, 1.03), (0.9595960, 2.0, 2.1)]
        ),
        0.1,
        10.0,
        0.002,
    )
    assert result["status"] == "NOT VERIFIED"
    assert not result["normalized_tolerance_configured"]
