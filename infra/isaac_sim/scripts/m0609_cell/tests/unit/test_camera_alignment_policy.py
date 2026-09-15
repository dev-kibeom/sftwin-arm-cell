import numpy as np
import pytest

from scene_builder.camera_alignment_policy import (
    D455_NEAR_RIGID_POLICY,
    near_rigid_rotation,
)


def test_near_rigid_policy_accepts_live_scale_noise_and_derives_proper_rotation():
    raw = np.array(
        ((1.000001, 0.0000001, 0.0), (-0.0000001, 0.999999, 0.0), (0.0, 0.0, 1.0000005))
    )
    rotation, evidence = near_rigid_rotation(raw)
    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)
    assert (
        evidence["max_axis_norm_deviation"]
        < D455_NEAR_RIGID_POLICY["axis_norm_deviation_tolerance"]
    )


@pytest.mark.parametrize(
    "matrix, expected_reason",
    [
        (np.diag((1.001, 1.0, 1.0)), "axis norm deviation"),
        (
            np.array(((1.0, 0.001, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))),
            "orthogonality error",
        ),
        (np.diag((-1.0, 1.0, 1.0)), "determinant deviation or reflection"),
        (np.diag((1.000009, 0.999991, 1.0)), "non-uniform scale"),
    ],
)
def test_near_rigid_policy_rejects_material_affine_components(matrix, expected_reason):
    with pytest.raises(RuntimeError, match=expected_reason):
        near_rigid_rotation(matrix)
