from pathlib import Path

import pytest

from derive_golden_fixture import REQUIRED_TOPICS, read_capture
from repository import local_artifact_bag, repository_root


ROOT = repository_root(__file__)
SOURCE = local_artifact_bag(ROOT, "arm-cell-m1c-rgbd-runtime-2026-09-11")
DERIVED = Path(__file__).resolve().parents[1] / "fixtures/isaac_rgbd_nominal_v1/bag"


def test_derived_fixture_is_a_byte_and_storage_timestamp_subset_of_source():
    if not SOURCE.is_dir():
        pytest.skip("source capture is not available in this checkout")
    _, source = read_capture(SOURCE)
    _, derived = read_capture(DERIVED)
    for topic in REQUIRED_TOPICS:
        assert set(derived[topic]) <= set(source[topic]), topic
    assert len(derived["/camera/color/image_raw"]) == 8
    assert len(derived["/camera/aligned_depth_to_color/image_raw"]) == 8
