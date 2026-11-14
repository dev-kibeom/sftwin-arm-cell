"""Pure acceptance policy for the composed D455 Color camera transform."""

import json

import numpy as np


# D455 Color's live composed matrix showed only approximately 1e-6 numerical
# deviation from a rigid transform. This policy accepts that small USD/asset
# composition noise, but rejects real scale, shear, and reflections before any
# logical-camera pose is authored.
D455_NEAR_RIGID_POLICY = {
    "axis_norm_deviation_tolerance": 1.0e-5,
    "orthogonality_tolerance": 1.0e-5,
    "determinant_tolerance": 1.0e-5,
    "nonuniform_scale_tolerance": 1.0e-5,
}


def matrix_metrics(matrix):
    array = np.asarray(matrix, dtype=float)
    rows = array
    columns = array.T
    row_norms = np.linalg.norm(rows, axis=1)
    column_norms = np.linalg.norm(columns, axis=1)
    row_dots = rows @ rows.T
    column_dots = columns @ columns.T
    off_diagonal = ~np.eye(3, dtype=bool)
    return {
        "row_norms": row_norms,
        "column_norms": column_norms,
        "row_dot_products": row_dots,
        "column_dot_products": column_dots,
        "max_axis_norm_deviation": float(
            max(np.max(np.abs(row_norms - 1.0)), np.max(np.abs(column_norms - 1.0)))
        ),
        "max_orthogonality_error": float(
            max(
                np.max(np.abs(row_dots[off_diagonal])),
                np.max(np.abs(column_dots[off_diagonal])),
            )
        ),
        "max_nonuniform_scale": float(max(np.ptp(row_norms), np.ptp(column_norms))),
        "determinant": float(np.linalg.det(array)),
    }


def near_rigid_rotation(matrix, policy=D455_NEAR_RIGID_POLICY):
    """Accept bounded numerical drift and derive a proper rotation."""
    array = np.asarray(matrix, dtype=float)
    if array.shape != (3, 3) or not np.all(np.isfinite(array)):
        raise RuntimeError("D455 Color composed rotation must be a finite 3x3 matrix")
    metrics = matrix_metrics(array)
    failures = []
    if metrics["max_axis_norm_deviation"] > policy["axis_norm_deviation_tolerance"]:
        failures.append("axis norm deviation")
    if metrics["max_orthogonality_error"] > policy["orthogonality_tolerance"]:
        failures.append("orthogonality error (shear evidence)")
    if abs(metrics["determinant"] - 1.0) > policy["determinant_tolerance"]:
        failures.append("determinant deviation or reflection")
    if metrics["max_nonuniform_scale"] > policy["nonuniform_scale_tolerance"]:
        failures.append("non-uniform scale")
    evidence = {
        key: value.tolist() if hasattr(value, "tolist") else value
        for key, value in metrics.items()
    }
    if failures:
        raise RuntimeError(
            "D455 Color composed transform rejected by near-rigid policy: "
            + ", ".join(failures)
            + "; evidence="
            + json.dumps(evidence)
        )

    first = array[0] / np.linalg.norm(array[0])
    second = array[1] - np.dot(array[1], first) * first
    second_norm = np.linalg.norm(second)
    if second_norm == 0.0:
        raise RuntimeError("D455 Color composed transform has a degenerate second axis")
    second = second / second_norm
    third = np.cross(first, second)
    if np.dot(third, array[2]) <= 0.0:
        raise RuntimeError(
            "D455 Color composed transform does not preserve a right-handed basis"
        )
    rotation = np.vstack((first, second, third))
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12):
        raise RuntimeError(
            "D455 Color rigid rotation derivation did not produce determinant +1"
        )
    return rotation, evidence
