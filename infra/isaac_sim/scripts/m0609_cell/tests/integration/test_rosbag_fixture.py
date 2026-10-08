from pathlib import Path

import pytest

from bag_fixture import sha256
from repository import local_artifact_bag, repository_root


def test_user_baseline_db_hash_is_preserved():
    bag = (
        local_artifact_bag(
            repository_root(__file__), "arm-cell-m1c-rgbd-pre-camerainfo-2026-09-11"
        )
        / "rgbd_pre_1c_no_camerainfo_2026-09-11.db3"
    )

    if not bag.is_file():
        pytest.skip("user-owned protected baseline is not available in this checkout")

    assert sha256(bag) == (
        "33a3abbc4de60f21fe63b85340817063380f719c5b483447cce6a0422045f526"
    )
