"""Simulator-independent camera inspection math for M0609 diagnostics."""

import math


CAMERA_PATH = "/World/SF_Twin_Cell/Vision/Camera_Sensor"
D455_ROOT_PATH = "/World/SF_Twin_Cell/CameraRig/RealSense_D455"
D455_COLOR_CAMERA_NAME = "Camera_OmniVision_OV9782_Color"
ACTION_GRAPH_PATH = "/World/SF_Twin_Cell/ROS2/ActionGraph"
DEPTH_HELPER_PATH = ACTION_GRAPH_PATH + "/CameraHelperDepth"
DEFAULT_REFERENCE_PRIM = "/World/SF_Twin_Cell/AMR/Mockup/RawPart"
CAMERA_LINK_TO_OPTICAL_BASIS = (
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
    (1.0, 0.0, 0.0),
)
USD_CAMERA_TO_ROS_OPTICAL_POINT_BASIS = (
    (1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
)


def is_rigid_rotation(matrix, tolerance=1e-8):
    """Return whether the 3x3 matrix has orthonormal axes and determinant +1."""
    rows = matrix
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))
    if any(abs(dot(row, row) - 1.0) > tolerance for row in rows):
        return False
    if any(abs(dot(rows[i], rows[j])) > tolerance for i in range(3) for j in range(i)):
        return False
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    return abs(determinant - 1.0) <= tolerance


def quaternion_xyzw_from_rotation(matrix):
    """Convert a verified rigid 3x3 rotation to an xyzw quaternion."""
    if not is_rigid_rotation(matrix):
        raise ValueError("camera transform contains scale, shear, or reflection")
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            (m[2][1] - m[1][2]) / scale,
            (m[0][2] - m[2][0]) / scale,
            (m[1][0] - m[0][1]) / scale,
            0.25 * scale,
        )
    index = max(range(3), key=lambda i: m[i][i])
    if index == 0:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return (
            0.25 * scale,
            (m[0][1] + m[1][0]) / scale,
            (m[0][2] + m[2][0]) / scale,
            (m[2][1] - m[1][2]) / scale,
        )
    if index == 1:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return (
            (m[0][1] + m[1][0]) / scale,
            0.25 * scale,
            (m[1][2] + m[2][1]) / scale,
            (m[0][2] - m[2][0]) / scale,
        )
    scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return (
        (m[0][2] + m[2][0]) / scale,
        (m[1][2] + m[2][1]) / scale,
        0.25 * scale,
        (m[1][0] - m[0][1]) / scale,
    )


def transpose(matrix):
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def multiply(left, right):
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def matrix_vector(matrix, vector):
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def determinant_3x3(matrix):
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def affine_3x3_diagnostics(matrix):
    """Report affine evidence without extracting or modifying a rotation.

    For a non-rigid matrix a scale/shear factorization depends on the selected
    convention.  The Gram matrix, axis lengths, and normalized axis dot
    products expose the same evidence without silently orthonormalizing it.
    """
    rows = tuple(tuple(float(value) for value in row) for row in matrix)
    columns = transpose(rows)
    dot = lambda left, right: sum(a * b for a, b in zip(left, right))
    norm = lambda vector: math.sqrt(dot(vector, vector))
    row_norms = [norm(row) for row in rows]
    column_norms = [norm(column) for column in columns]
    row_dots = [[dot(left, right) for right in rows] for left in rows]
    column_dots = [[dot(left, right) for right in columns] for left in columns]
    normalized_column_dots = [
        [
            (
                None
                if column_norms[i] == 0.0 or column_norms[j] == 0.0
                else column_dots[i][j] / (column_norms[i] * column_norms[j])
            )
            for j in range(3)
        ]
        for i in range(3)
    ]
    return {
        "upper_left_3x3": [list(row) for row in rows],
        "row_norms": row_norms,
        "column_norms": column_norms,
        "row_dot_products": row_dots,
        "column_dot_products": column_dots,
        "determinant": determinant_3x3(rows),
        "gram_matrix_column_metric": column_dots,
        "normalized_column_dot_products": normalized_column_dots,
        "is_rigid_rotation": is_rigid_rotation(rows),
        "scale_shear_decomposition": {
            "status": "NOT VERIFIED" if not is_rigid_rotation(rows) else "VERIFIED",
            "reason": (
                None
                if is_rigid_rotation(rows)
                else "non-rigid affine matrix has no unique scale/shear decomposition without selecting an orthonormalization convention; raw Gram-matrix evidence is retained"
            ),
        },
    }


def matrix_difference(left, right):
    difference = [
        [float(left[row][column]) - float(right[row][column]) for column in range(4)]
        for row in range(4)
    ]
    return {
        "matrix_difference": difference,
        "max_abs_element_error": max(abs(value) for row in difference for value in row),
    }


def rotation_angle_rad(rotation):
    """Return the principal angle of a verified 3x3 rotation matrix."""
    if not is_rigid_rotation(rotation):
        raise ValueError("camera transform contains scale, shear, or reflection")
    cosine = max(
        -1.0, min(1.0, (sum(rotation[index][index] for index in range(3)) - 1.0) / 2.0)
    )
    return math.acos(cosine)


def pose_difference(
    reference_rotation, reference_translation, candidate_rotation, candidate_translation
):
    """Express candidate pose relative to the reference camera-local axes.

    Both rotations map their respective local coordinate columns into world
    coordinates.  This function is simulator-independent and deliberately
    compares raw composed USD local frames; it does not infer rendering-axis
    conventions or calibration equivalence.
    """
    if not is_rigid_rotation(reference_rotation) or not is_rigid_rotation(
        candidate_rotation
    ):
        raise ValueError("camera transform contains scale, shear, or reflection")
    reference_from_world = transpose(reference_rotation)
    delta_world = tuple(
        candidate_translation[index] - reference_translation[index]
        for index in range(3)
    )
    relative_rotation = multiply(reference_from_world, candidate_rotation)
    return {
        "translation_delta_world_m": list(delta_world),
        "translation_delta_in_reference_local_m": list(
            matrix_vector(reference_from_world, delta_world)
        ),
        "rotation_delta_rad": rotation_angle_rad(relative_rotation),
        "relative_rotation_matrix": [list(row) for row in relative_rotation],
    }


def usd_camera_to_optical(point):
    """Convert USD camera-local point coordinates to ROS optical coordinates.

    The composed Camera_Sensor viewport verifies the standard UsdGeom.Camera
    local axes here: +X right, +Y up, and -Z forward.  This is a point
    coordinate conversion, not a change to the camera prim pose.
    """
    return tuple(
        sum(
            USD_CAMERA_TO_ROS_OPTICAL_POINT_BASIS[row][column] * point[column]
            for column in range(3)
        )
        for row in range(3)
    )


def normalized_dot(left, right):
    magnitude = lambda value: math.sqrt(
        sum(component * component for component in value)
    )
    return sum(a * b for a, b in zip(left, right)) / (
        magnitude(left) * magnitude(right)
    )


def ray_direction(origin, target):
    delta = tuple(target[index] - origin[index] for index in range(3))
    magnitude = math.sqrt(sum(component * component for component in delta))
    if magnitude == 0.0:
        raise ValueError("reference point equals camera origin")
    return tuple(component / magnitude for component in delta), magnitude
