import math
from dataclasses import replace
from types import SimpleNamespace

import gripper_runtime.grasp_attachment as grasp_attachment
import gripper_runtime.grasp_joint as grasp_joint
import gripper_runtime.grasp_stage as grasp_stage
import pytest
from gripper_runtime.capture_diagnostics import BoundedCaptureDiagnosticWriter
from gripper_runtime.grasp_attachment import GraspManager, HoldingState
from gripper_runtime.grasp_qualification import ROBOTIQ_2F_85_MODEL_ID
from gripper_runtime.grasp_joint import UsdFixedJointOwner
from gripper_runtime.grasp_policy import (
    CaptureCandidate,
    CaptureConfig,
    CapturePolicy,
)
from gripper_runtime.grasp_stage import (
    CaptureReferenceConfig,
    EligibleObjectRegistry,
    StageCaptureQuery,
)


def capture_config():
    return CaptureConfig(
        capture_volume_dimensions_m=(0.10, 0.10, 0.10),
    )


def valid_candidate(path="/World/Cube"):
    return CaptureCandidate(
        prim_path=path,
        eligible=True,
        tcp_local_position_m=(0.0, 0.0, 0.0),
        position_error_m=0.005,
        orientation_error_deg=2.0,
        measured_opening_mm=12.0,
        shape="box",
    )


class _BasisTransform:
    def __init__(
        self,
        x_axis=(1.0, 0.0, 0.0),
        y_axis=(0.0, 1.0, 0.0),
        z_axis=(0.0, 0.0, 1.0),
    ):
        self.axes = (x_axis, y_axis, z_axis)

    def TransformDir(self, axis):
        return tuple(
            sum(self.axes[index][component] * axis[index] for index in range(3))
            for component in range(3)
        )

    def __rmul__(self, vector):
        return tuple(
            sum(
                float(vector[index]) * self.axes[index][component] for index in range(3)
            )
            for component in range(3)
        )


class _Quaternion:
    def __init__(self, real=1.0, imaginary=(0.0, 0.0, 0.0)):
        self.real = float(real)
        self.imaginary = tuple(float(value) for value in imaginary)

    def __mul__(self, other):
        ax, ay, az = self.imaginary
        bx, by, bz = other.imaginary
        return _Quaternion(
            self.real * other.real - ax * bx - ay * by - az * bz,
            (
                self.real * bx + other.real * ax + ay * bz - az * by,
                self.real * by + other.real * ay + az * bx - ax * bz,
                self.real * bz + other.real * az + ax * by - ay * bx,
            ),
        )

    def Normalize(self):
        norm = math.sqrt(self.real**2 + sum(value**2 for value in self.imaginary))
        self.real /= norm
        self.imaginary = tuple(value / norm for value in self.imaginary)

    def GetReal(self):
        return self.real

    def GetImaginary(self):
        return self.imaginary

    def GetInverse(self):
        return _Quaternion(self.real, tuple(-value for value in self.imaginary))


class _RigidWorldTransform:
    def __init__(self, translation, rotation):
        self.translation = tuple(float(value) for value in translation)
        self.rotation = rotation

    def ExtractTranslation(self):
        return self.translation

    def ExtractRotationQuat(self):
        return self.rotation

    def ExtractRotationMatrix(self):
        return _RotatedMatrix(self.rotation)

    def TransformDir(self, axis):
        x, y, z = axis
        qx, qy, qz = self.rotation.imaginary
        qw = self.rotation.real
        # q * v * conjugate(q)
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + qy * tz - qz * ty,
            y + qw * ty + qz * tx - qx * tz,
            z + qw * tz + qx * ty - qy * tx,
        )

    def Transform(self, point):
        rotated = self.TransformDir(point)
        return tuple(rotated[index] + self.translation[index] for index in range(3))

    def GetInverse(self):
        class _Inverse:
            def Transform(_self, point):
                del point
                return (0.0, 0.0, 0.0)

        return _Inverse()


class _RotatedMatrix:
    def __init__(self, rotation):
        self.rotation = rotation

    def __rmul__(self, vector):
        transform = _RigidWorldTransform((0.0, 0.0, 0.0), self.rotation)
        return transform.TransformDir(vector)


class _FakeGf:
    Vec3d = staticmethod(lambda *values: tuple(values))
    Quatd = staticmethod(lambda real, imaginary: _Quaternion(real, imaginary))
    Matrix3d = staticmethod(_RotatedMatrix)


def test_capture_reference_pose_is_transformed_from_named_frame_to_stage_world(
    monkeypatch,
):
    monkeypatch.setattr(
        grasp_stage, "_pxr_modules", lambda: (_FakeGf, None, None, None, None)
    )
    half = math.sqrt(0.5)
    frame_world = _RigidWorldTransform(
        (1.0, -2.0, 0.5), _Quaternion(half, (0.0, 0.0, half))
    )

    position_world, orientation_world = StageCaptureQuery._transform_pose_to_world(
        frame_world,
        (0.2, 0.3, 0.4),
        (0.0, 0.0, 0.0, 1.0),
    )

    assert position_world == pytest.approx((0.7, -1.8, 0.9))
    assert orientation_world == pytest.approx((0.0, 0.0, half, half))


def test_expected_tcp_orientation_compares_in_transformed_world_frame(monkeypatch):
    monkeypatch.setattr(
        grasp_stage, "_pxr_modules", lambda: (_FakeGf, None, None, None, None)
    )
    half = math.sqrt(0.5)
    tcp_world = _RigidWorldTransform(
        (1.0, -2.0, 0.5), _Quaternion(half, (0.0, 0.0, half))
    )
    _, orientation_world = StageCaptureQuery._transform_pose_to_world(
        tcp_world, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    )

    assert StageCaptureQuery._expected_tcp_orientation_error_deg(
        tcp_world, orientation_world, symmetry_order=2
    ) == pytest.approx(0.0)


@pytest.mark.parametrize("order", [0, 1, 2, 4])
def test_capture_query_compares_transformed_reference_position_and_orientation(
    monkeypatch,
    order,
):
    monkeypatch.setattr(
        grasp_stage, "_pxr_modules", lambda: (_FakeGf, None, None, None, None)
    )
    half = math.sqrt(0.5)
    frame_world = _RigidWorldTransform(
        (1.0, -2.0, 0.5), _Quaternion(half, (0.0, 0.0, half))
    )

    class _Prim:
        def IsValid(self):
            return True

        def GetAttribute(self, name):
            return (
                SimpleNamespace(Get=lambda: "box") if name == "sf_twin:shape" else None
            )

    class _Stage:
        def GetPrimAtPath(self, _path):
            return _Prim()

    class _Registry:
        def candidate_paths(self, _stage):
            return ["/robot/RawPart"]

    world_tcp = _RigidWorldTransform(
        (0.7, -1.8, 0.9), _Quaternion(half, (0.0, 0.0, half))
    )
    query = StageCaptureQuery(
        _Stage(),
        "/robot/tcp",
        _Registry(),
        SimpleNamespace(robot_root_path="/robot", get_width=lambda: 70.0),
    )
    assert query.configure_expected_tcp_reference(
        "base_link", (0.2, 0.3, 0.4), (0.0, 0.0, 0.0, 1.0), order, 70.0
    )
    query._world_transform = lambda path: (
        frame_world if path == "/robot/base_link" else world_tcp
    )

    available, candidates = query.query()

    assert available is True
    assert candidates[0].position_error_m == pytest.approx(0.0)
    assert candidates[0].expected_orientation_error_deg == pytest.approx(0.0)
    assert candidates[0].shape == "box"


def test_capture_query_reports_missing_named_frame_without_blocking_capture(
    monkeypatch,
):
    class _Prim:
        def __init__(self, valid):
            self.valid = valid

        def IsValid(self):
            return self.valid

    class _Stage:
        def GetPrimAtPath(self, path):
            return _Prim(path == "/robot/tcp")

    query = StageCaptureQuery(
        _Stage(),
        "/robot/tcp",
        EligibleObjectRegistry(),
        SimpleNamespace(robot_root_path="/robot"),
    )
    query.reference_config = object()
    query.expected_tcp_reference = (
        "base_link",
        (0.1, 0.2, 0.3),
        (0.0, 0.0, 0.0, 1.0),
        2,
        70.0,
    )
    query._world_transform = lambda _path: _RigidWorldTransform(
        (0.0, 0.0, 0.0), _Quaternion()
    )
    monkeypatch.setattr(
        StageCaptureQuery,
        "_canonical_capture_reference",
        staticmethod(lambda *_args: pytest.fail("canonical fallback was used")),
    )

    available, candidates = query.query()

    assert available is True
    assert candidates == []
    assert (
        query.reference_diagnostics()["rejection_reason"] == "reference_frame_missing"
    )


def test_capture_query_keeps_invalid_reference_transform_diagnostic_only(
    monkeypatch,
):
    class _ScaledWorldTransform(_RigidWorldTransform):
        def TransformDir(self, axis):
            return (2.0 * axis[0], axis[1], axis[2])

    class _Prim:
        def IsValid(self):
            return True

        def GetAttribute(self, _name):
            return None

    class _Stage:
        def GetPrimAtPath(self, _path):
            return _Prim()

    class _Registry:
        def candidate_paths(self, _stage):
            return []

    query = StageCaptureQuery(
        _Stage(), "/robot/tcp", _Registry(), SimpleNamespace(robot_root_path="/robot")
    )
    assert query.configure_expected_tcp_reference(
        "base_link", (0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0), 2, 70.0
    )
    query._world_transform = lambda path: (
        _RigidWorldTransform((1.0, 2.0, 3.0), _Quaternion())
        if path == "/robot/tcp"
        else _ScaledWorldTransform()
    )

    available, candidates = query.query()

    assert available is True
    assert candidates == []
    assert (
        query.reference_diagnostics()["rejection_reason"]
        == "reference_frame_transform_invalid"
    )


def test_expected_tcp_orientation_uses_matrix_vector_product(monkeypatch):
    class FakeMatrix3d:
        def __init__(self, _quaternion):
            self.basis = _BasisTransform()

        def __rmul__(self, vector):
            return vector * self.basis

    class FakeGf:
        Vec3d = staticmethod(lambda *values: tuple(values))
        Quatd = staticmethod(lambda real, imaginary: (real, tuple(imaginary)))
        Matrix3d = FakeMatrix3d

    monkeypatch.setattr(
        grasp_stage, "_pxr_modules", lambda: (FakeGf, None, None, None, None)
    )

    error = StageCaptureQuery._expected_tcp_orientation_error_deg(
        SimpleNamespace(ExtractRotationMatrix=lambda: _BasisTransform()),
        (0.0, 0.0, 0.0, 1.0),
        symmetry_order=2,
    )

    assert error == 0.0


def test_rawpart_side_drop_uses_actual_up_local_x_for_top_face():
    rawpart = _BasisTransform(
        x_axis=(0.0, 0.0, 0.07),
        y_axis=(0.0, 0.08, 0.0),
        z_axis=(0.07, 0.0, 0.0),
    )
    tcp = _BasisTransform(
        x_axis=(1.0, 0.0, 0.0),
        y_axis=(0.0, 1.0, 0.0),
        z_axis=(0.0, 0.0, -1.0),
    )

    error = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp,
        rawpart,
        expected_contact_width_mm=70.0,
        orientation_symmetry_order=2,
        orientation_tolerance_deg=5.0,
        contact_width_tolerance_mm=2.0,
    )

    assert error <= 5.0


def test_rawpart_rectangle_rejects_changed_object_direction_but_allows_axis_sign():
    rawpart = _BasisTransform(
        x_axis=(0.0, 0.08, 0.0),
        y_axis=(0.07, 0.0, 0.0),
        z_axis=(0.0, 0.0, 0.07),
    )
    matching_tcp = _BasisTransform(
        x_axis=(0.0, 1.0, 0.0),
        y_axis=(1.0, 0.0, 0.0),
        z_axis=(0.0, 0.0, -1.0),
    )
    ninety_degree_tcp = _BasisTransform(
        x_axis=(1.0, 0.0, 0.0),
        y_axis=(0.0, -1.0, 0.0),
        z_axis=(0.0, 0.0, -1.0),
    )

    matching = StageCaptureQuery._rawpart_orientation_error_deg(
        matching_tcp, rawpart, 80.0, 2, 5.0, 2.0
    )
    turned = StageCaptureQuery._rawpart_orientation_error_deg(
        ninety_degree_tcp, rawpart, 80.0, 2, 5.0, 2.0
    )

    assert matching <= 5.0
    assert turned > 5.0
    reversed_jaw = _BasisTransform(
        x_axis=(0.0, -1.0, 0.0),
        y_axis=(-1.0, 0.0, 0.0),
        z_axis=(0.0, 0.0, -1.0),
    )
    assert (
        StageCaptureQuery._rawpart_orientation_error_deg(
            reversed_jaw, rawpart, 80.0, 2, 5.0, 2.0
        )
        <= 5.0
    )


def test_rawpart_rectangle_short_axis_uses_short_edge_width_and_rejects_other_width():
    rawpart = _BasisTransform(
        x_axis=(0.0, 0.08, 0.0),
        y_axis=(0.07, 0.0, 0.0),
        z_axis=(0.0, 0.0, 0.07),
    )
    short_edge_tcp = _BasisTransform(
        x_axis=(1.0, 0.0, 0.0),
        y_axis=(0.0, -1.0, 0.0),
        z_axis=(0.0, 0.0, -1.0),
    )

    assert (
        StageCaptureQuery._orientation_error_deg(
            short_edge_tcp,
            rawpart,
            expected_contact_width_mm=70.0,
            orientation_symmetry_order=2,
            orientation_tolerance_deg=5.0,
            contact_width_tolerance_mm=2.0,
        )
        <= 5.0
    )
    assert (
        StageCaptureQuery._orientation_error_deg(
            short_edge_tcp,
            rawpart,
            expected_contact_width_mm=75.0,
            orientation_symmetry_order=2,
            orientation_tolerance_deg=5.0,
            contact_width_tolerance_mm=2.0,
        )
        is None
    )


def test_rawpart_capture_uses_authored_dimensions_with_unscaled_rigidbody():
    tcp_world = _BasisTransform(
        x_axis=(0.0, 0.0, 1.0),
        y_axis=(0.0, 1.0, 0.0),
        z_axis=(-1.0, 0.0, 0.0),
    )
    rawpart_world = _BasisTransform()

    error, reason = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp_world,
        rawpart_world,
        80.0,
        2,
        5.0,
        2.0,
        dimensions_m=(0.07, 0.07, 0.08),
        return_reason=True,
    )

    assert error == pytest.approx(0.0)
    assert reason is None


def test_rawpart_dimensions_do_not_change_transform_axis_orthogonality():
    # The transform axes have a small, acceptable dot product. Dividing them
    # by the much smaller authored dimensions would incorrectly fail the test.
    rawpart_world = _BasisTransform(
        x_axis=(1.0, 0.0, 0.0),
        y_axis=(1e-5, math.sqrt(1.0 - 1e-10), 0.0),
        z_axis=(0.0, 0.0, 1.0),
    )
    tcp_world = _BasisTransform(
        x_axis=(0.0, 0.0, 1.0),
        y_axis=(0.0, 1.0, 0.0),
        z_axis=(-1.0, 0.0, 0.0),
    )

    small_dimensions = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp_world,
        rawpart_world,
        80.0,
        2,
        5.0,
        2.0,
        dimensions_m=(0.07, 0.07, 0.08),
        return_reason=True,
    )
    different_dimensions = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp_world,
        rawpart_world,
        80.0,
        2,
        5.0,
        2.0,
        dimensions_m=(0.06, 0.09, 0.08),
        return_reason=True,
    )

    assert small_dimensions[0] == pytest.approx(0.0)
    assert different_dimensions[0] == pytest.approx(0.0)
    assert small_dimensions[1] is None
    assert different_dimensions[1] is None


def test_authored_rawpart_dimensions_are_read_from_the_rigidbody_prim():
    class _Attribute:
        def IsValid(self):
            return True

        def Get(self):
            return (0.07, 0.07, 0.08)

    prim = SimpleNamespace(GetAttribute=lambda name: _Attribute())

    assert StageCaptureQuery._authored_dimensions(prim) == (0.07, 0.07, 0.08)


def test_rawpart_capture_allows_tilted_side_and_top_bottom_opposing_faces():
    rawpart = _BasisTransform(
        x_axis=(0.07, 0.0, 0.0),
        y_axis=(0.0, 0.05, 0.0),
        z_axis=(0.0, 0.0, 0.08),
    )
    tilted_side_tcp = _BasisTransform(
        x_axis=(1.0, 0.0, 0.0),
        y_axis=(0.0, 0.0, 1.0),
        z_axis=(0.0, -1.0, 0.0),
    )
    top_bottom_tcp = _BasisTransform(
        x_axis=(0.0, 0.0, 1.0),
        y_axis=(1.0, 0.0, 0.0),
        z_axis=(0.0, 1.0, 0.0),
    )
    assert (
        StageCaptureQuery._rawpart_orientation_error_deg(
            tilted_side_tcp, rawpart, 70.0, 2, 5.0, 2.0
        )
        <= 5.0
    )
    assert (
        StageCaptureQuery._rawpart_orientation_error_deg(
            top_bottom_tcp, rawpart, 80.0, 2, 5.0, 2.0
        )
        <= 5.0
    )


def test_rawpart_square_allows_ninety_degrees_but_rectangle_does_not():
    square = _BasisTransform(
        x_axis=(0.0, 0.0, 0.07),
        y_axis=(0.07, 0.0, 0.0),
        z_axis=(0.0, 0.07, 0.0),
    )
    rotated_tcp = _BasisTransform(
        x_axis=(1.0, 0.0, 0.0),
        y_axis=(0.0, 0.0, 1.0),
        z_axis=(0.0, 0.0, -1.0),
    )

    square_error = StageCaptureQuery._rawpart_orientation_error_deg(
        rotated_tcp, square, 70.0, 4, 5.0, 2.0
    )

    assert square_error <= 5.0


def test_rawpart_capture_rejects_nonorthogonal_rawpart_frame():
    ambiguous = _BasisTransform(
        x_axis=(0.0, 0.0, 0.07),
        y_axis=(0.0, 0.07, 0.0),
        z_axis=(0.0, 0.0, 0.08),
    )
    tcp = _BasisTransform(z_axis=(0.0, 0.0, -1.0))

    error = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp, ambiguous, 70.0, 2, 5.0, 2.0
    )

    assert error is None


@pytest.mark.parametrize(
    "dimensions,order,expected_error",
    [
        ((0.08, 0.07, 0.07), 2, 0.0),
        ((0.07, 0.07, 0.08), 4, 0.0),
        ((0.07, 0.07, 0.08), 2, 0.0),
        ((0.08, 0.07, 0.08), 4, 0.0),
    ],
)
def test_rawpart_direction_accepts_observed_symmetry_independent_of_local_xy(
    dimensions, order, expected_error
):
    rawpart = _BasisTransform(
        x_axis=(dimensions[0], 0.0, 0.0),
        y_axis=(0.0, dimensions[1], 0.0),
        z_axis=(0.0, 0.0, dimensions[2]),
    )
    tcp = _BasisTransform(x_axis=(1.0, 0.0, 0.0))

    error, reason = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp,
        rawpart,
        dimensions[0] * 1000.0,
        order,
        5.0,
        2.0,
        return_reason=True,
    )

    if expected_error is None:
        assert error is None
        assert reason == "rawpart_orientation_invalid"
    else:
        assert error == pytest.approx(expected_error)
        assert reason is None


def test_horizontal_rawpart_accepts_tilted_tcp_closing_axis_on_opposing_faces():
    angle = math.radians(30.0)
    tcp = _BasisTransform(
        x_axis=(0.0, 1.0, 0.0),
        y_axis=(math.cos(angle), 0.0, math.sin(angle)),
        z_axis=(math.sin(angle), 0.0, -math.cos(angle)),
    )
    rawpart = _BasisTransform(
        x_axis=(0.0, 0.07, 0.0),
        y_axis=(0.08, 0.0, 0.0),
        z_axis=(0.0, 0.0, 0.07),
    )

    error = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp, rawpart, 70.0, 2, 5.0, 2.0
    )

    assert error == pytest.approx(0.0)


def test_tilted_rawpart_and_tcp_accept_aligned_closing_axis():
    angle = math.radians(30.0)
    rawpart = _BasisTransform(
        x_axis=(0.012, 0.0, 0.0),
        y_axis=(0.0, 0.025 * math.cos(angle), 0.025 * math.sin(angle)),
        z_axis=(0.0, -0.030 * math.sin(angle), 0.030 * math.cos(angle)),
    )
    tcp = _BasisTransform(
        x_axis=(1.0, 0.0, 0.0),
        y_axis=(0.0, math.cos(angle), math.sin(angle)),
        z_axis=(0.0, -math.sin(angle), math.cos(angle)),
    )

    error = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp, rawpart, 12.0, 2, 5.0, 2.0
    )

    assert error == pytest.approx(0.0)


def test_tilted_tcp_closing_axis_misaligned_with_contact_faces_is_rejected():
    angle = math.radians(30.0)
    rawpart = _BasisTransform(
        x_axis=(0.012, 0.0, 0.0),
        y_axis=(0.0, 0.025 * math.cos(angle), 0.025 * math.sin(angle)),
        z_axis=(0.0, -0.030 * math.sin(angle), 0.030 * math.cos(angle)),
    )
    tcp = _BasisTransform(
        x_axis=(0.0, math.cos(angle), math.sin(angle)),
        y_axis=(1.0, 0.0, 0.0),
        z_axis=(0.0, math.sin(angle), -math.cos(angle)),
    )

    error = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp, rawpart, 12.0, 2, 5.0, 2.0
    )

    assert error == pytest.approx(90.0)


def test_observed_symmetry_order_is_independent_of_rawpart_local_dimensions():
    angle = math.radians(35.0)
    rawpart = _BasisTransform(
        x_axis=(0.08 * math.cos(angle), 0.08 * math.sin(angle), 0.0),
        y_axis=(-0.07 * math.sin(angle), 0.07 * math.cos(angle), 0.0),
        z_axis=(0.0, 0.0, 0.07),
    )
    tcp = _BasisTransform(
        x_axis=(math.cos(angle), math.sin(angle), 0.0),
        y_axis=(-math.sin(angle), math.cos(angle), 0.0),
        z_axis=(0.0, 0.0, -1.0),
    )

    valid, valid_reason = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp, rawpart, 80.0, 2, 5.0, 2.0, return_reason=True
    )
    invalid, invalid_reason = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp, rawpart, 80.0, 4, 5.0, 2.0, return_reason=True
    )

    assert valid == pytest.approx(0.0)
    assert valid_reason is None
    assert invalid == pytest.approx(0.0)
    assert invalid_reason is None


def test_orientation_ignores_raw_frame_rotation_when_grasp_axes_align():
    tcp = _BasisTransform()
    cube = _BasisTransform(
        x_axis=(0.0, 1.0, 0.0),
        y_axis=(-1.0, 0.0, 0.0),
    )

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) > 5.0


def test_orientation_accepts_ninety_degree_cube_top_face_symmetry():
    tcp = _BasisTransform(z_axis=(0.0, 0.0, -1.0))
    cube = _BasisTransform(
        x_axis=(0.0, 1.0, 0.0),
        y_axis=(-1.0, 0.0, 0.0),
    )

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) <= 5.0


def test_orientation_rejects_approach_axis_outside_tolerance():
    tcp = _BasisTransform(z_axis=(0.0, 0.70710678, 0.70710678))
    cube = _BasisTransform()

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) > 5.0


def test_orientation_rejects_reversed_top_approach():
    tcp = _BasisTransform()
    cube = _BasisTransform()

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) > 5.0


class _PoseTransform(_BasisTransform):
    def __init__(self, position=(0.0, 0.0, 0.0), **kwargs):
        super().__init__(**kwargs)
        self.position = position

    def ExtractTranslation(self):
        return self.position


def test_canonical_capture_reference_uses_top_surface_and_grasp_offset():
    cube = _PoseTransform(position=(1.0, 2.0, 3.0))
    config = CaptureReferenceConfig(
        fixture_dimensions_m=(0.07, 0.07, 0.08),
        observation_to_grasp_tcp_m=-0.0025,
        insertion_axis_tcp=(0.0, 0.0, 1.0),
    )

    reference = StageCaptureQuery._canonical_capture_reference(cube, config)

    assert reference.position == (1.0, 2.0, 3.0375)
    assert reference.top_normal == (0.0, 0.0, 1.0)


def test_capture_reference_config_rejects_invalid_geometry():
    with pytest.raises(ValueError):
        CaptureReferenceConfig(
            fixture_dimensions_m=(0.07, 0.07, 0.0),
            observation_to_grasp_tcp_m=-0.0025,
            insertion_axis_tcp=(0.0, 0.0, 1.0),
        ).validate()


def test_orientation_rejects_closing_axis_outside_tolerance():
    tcp = _BasisTransform(x_axis=(0.0, 0.70710678, 0.70710678))
    cube = _BasisTransform()

    assert StageCaptureQuery._orientation_error_deg(tcp, cube) > 5.0


class FakeGripper:
    MODEL_ID = ROBOTIQ_2F_85_MODEL_ID

    def __init__(self):
        self.state = SimpleNamespace(value="closing")
        self.width_mm = 12.0
        self.stop_calls = 0
        self.cancel_motion_calls = 0

    def get_width(self):
        return self.width_mm

    def stop(self):
        self.stop_calls += 1
        self.state.value = "holding"

    def open(self, duration=None):
        del duration
        self.state.value = "opening"

    def finish_open(self):
        self.state.value = "holding"
        self.width_mm = 85.0

    def cancel_motion(self):
        self.cancel_motion_calls += 1
        self.state.value = "idle"


class FakeCandidateSource:
    def __init__(self, candidates):
        self.candidates = candidates
        self.runtime_available = True
        self.expected_tcp_reference = None
        self.query_calls = 0

    def configure_expected_tcp_reference(
        self,
        frame_id,
        position,
        orientation_xyzw,
        symmetry_order,
        expected_contact_width_mm=None,
    ):
        self.expected_tcp_reference = (
            frame_id,
            tuple(position),
            tuple(orientation_xyzw),
            symmetry_order,
            expected_contact_width_mm,
        )
        return True

    def query(self):
        self.query_calls += 1
        return self.runtime_available, list(self.candidates)


class FakeJointOwner:
    def __init__(self):
        self.owned_path = None
        self.created = []
        self.removed = []
        self.foreign_paths = {"/World/ForeignJoint"}
        self.valid = True
        self.target_exists = True
        self.remove_result = True

    def create(self, target_path):
        self.owned_path = f"{target_path}/sf_scripted_grasp_joint"
        self.created.append((self.owned_path, target_path))
        return self.owned_path

    def remove_owned(self):
        if self.owned_path is not None:
            self.removed.append(self.owned_path)
            if self.remove_result:
                self.owned_path = None
            return self.remove_result
        return False

    def owned_joint_valid(self):
        return self.valid and self.owned_path is not None and self.target_exists

    def has_owned_joint(self):
        return self.owned_path is not None


class FakePreCaptureStability:
    def __init__(self, joint_owner, result=True):
        self.joint_owner = joint_owner
        self.result = result
        self.calls = []

    def release_for_capture(self, target_path):
        self.calls.append((target_path, self.joint_owner.has_owned_joint()))
        return self.result


class ExistingUnownedJointOwner(FakeJointOwner):
    def __init__(self):
        super().__init__()
        self.existing_joint = True

    def create(self, target_path):
        del target_path
        return False


def test_existing_unowned_joint_does_not_silently_report_held():
    source = FakeCandidateSource([valid_candidate()])
    joints = ExistingUnownedJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_created_joint_without_confirmed_ownership_fails_closed():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    joints.valid = False
    instance = manager(source, joints)

    instance.update(0.01)

    assert len(joints.created) == 1
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_shutdown_cleanup_failure_invalidates_runtime_and_preserves_joint():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    joints.remove_result = False

    assert instance.shutdown() is False
    assert joints.has_owned_joint() is True
    assert instance.ready is False
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_timeline_reset_removes_attachment_without_actuating_stopped_simulation():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.HELD

    assert instance.reset_for_simulation_stop() is True

    assert joints.has_owned_joint() is False
    assert instance.gripper.cancel_motion_calls == 1
    assert instance.gripper.state.value == "idle"
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert instance.capture_reference_id is None


def test_repeated_close_open_cycle_reaches_held_and_released_deterministically():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    unknown = instance.holding_observation()
    assert unknown.sequence > 0
    instance.update(0.01)
    held = instance.holding_observation()
    assert held.state is HoldingState.HELD
    assert held.sequence > unknown.sequence

    instance.gripper.finish_open()
    instance.release(open_gripper=False)
    released = instance.holding_observation()
    assert released.state is HoldingState.RELEASED
    assert released.sequence > held.sequence

    instance.gripper.state.value = "holding"
    instance.arm_close_transaction()
    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)
    held_again = instance.holding_observation()
    assert held_again.state is HoldingState.HELD
    assert held_again.sequence > released.sequence


def test_holding_sequence_changes_only_when_semantic_state_changes():
    instance = manager(FakeCandidateSource([valid_candidate()]), FakeJointOwner())

    instance.update(0.01)
    held = instance.holding_observation()
    assert held.state is HoldingState.HELD

    instance.update(0.01)
    repeated_held = instance.holding_observation()
    assert repeated_held.state is HoldingState.HELD
    assert repeated_held.sequence == held.sequence

    instance.gripper.finish_open()
    instance.release(open_gripper=False)
    released = instance.holding_observation()
    assert released.state is HoldingState.RELEASED
    assert released.sequence > repeated_held.sequence

    instance.ready = False
    instance.update(0.01)
    unknown = instance.holding_observation()
    assert unknown.state is HoldingState.UNKNOWN
    assert unknown.sequence > released.sequence


def test_runtime_readiness_change_advances_holding_sequence():
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    ready_unknown = instance.holding_observation()

    instance.ready = False
    instance.update(0.01)
    unavailable_unknown = instance.holding_observation()

    assert unavailable_unknown.state is HoldingState.UNKNOWN
    assert unavailable_unknown.sequence > ready_unknown.sequence


def test_applied_gripper_command_starts_new_observation_even_when_state_is_same():
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    instance.gripper.finish_open()
    instance.update(0.01)
    released_before_command = instance.holding_observation()
    assert released_before_command.state is HoldingState.RELEASED

    instance.note_gripper_command_applied()
    released_after_command = instance.holding_observation()

    assert released_after_command.state is HoldingState.RELEASED
    assert released_after_command.sequence > released_before_command.sequence


def test_restored_holding_sequence_advances_past_previous_runtime_value():
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)

    instance.restore_holding_sequence(24614)
    instance.gripper.finish_open()
    instance.update(0.01)

    observation = instance.holding_observation()
    assert observation.state is HoldingState.RELEASED
    assert observation.sequence == 24615


def test_selected_motion_tcp_and_contact_width_become_capture_reference():
    source = FakeCandidateSource([])
    instance = manager(source, FakeJointOwner(), arm=False)
    values = {
        "motion_goal_uuid": "pick-frame-aware",
        "frame_id": "base_link",
        "values": (
            0.1,
            0.2,
            0.3,
            0.0,
            0.0,
            0.7071067811865476,
            0.7071067811865476,
            80.0,
            2.0,
        ),
    }

    assert instance.configure_selected_grasp_reference(values)
    assert source.expected_tcp_reference == (
        "base_link",
        values["values"][:3],
        values["values"][3:7],
        2,
        80.0,
    )
    assert instance.config.capture_volume_dimensions_m == (0.10, 0.10, 0.10)


def test_close_transaction_keeps_motion_goal_as_diagnostic_correlation_only():
    source = FakeCandidateSource([])
    instance = manager(source, FakeJointOwner(), arm=False)
    values = (
        0.1,
        0.2,
        0.3,
        0.0,
        0.0,
        0.0,
        1.0,
        80.0,
        2.0,
    )

    assert instance.configure_selected_grasp_reference(
        {
            "motion_goal_uuid": "pick-current",
            "values": values,
        }
    )
    assert instance.arm_close_transaction("pick-old") is True
    assert instance.arm_close_transaction("pick-current") is True


_DEFAULT_CONFIG = object()


def manager(source, joints, config=_DEFAULT_CONFIG, stability=None, arm=True):
    instance = GraspManager(
        gripper=FakeGripper(),
        tcp_path="/World/TCP",
        config=capture_config() if config is _DEFAULT_CONFIG else config,
        candidate_source=source,
        joint_owner=joints,
        pre_capture_stability=stability,
    )
    if arm:
        instance.arm_close_transaction()
        instance.gripper.state.value = "holding"
    return instance


class _Prim:
    def __init__(
        self,
        valid=True,
        type_name="",
        path=None,
        body0=None,
        body1=None,
    ):
        self.valid = valid
        self.type_name = type_name
        self.path = path
        self.body0 = body0
        self.body1 = body1

    def IsValid(self):
        return self.valid

    def GetAttribute(self, _name):
        return None

    def GetTypeName(self):
        return self.type_name

    def GetName(self):
        return str(self.path).rsplit("/", 1)[-1]

    def GetPath(self):
        return self.path

    def GetRelationship(self, name):
        return SimpleNamespace(
            GetTargets=lambda: [self.body0 if name == "physics:body0" else self.body1]
        )


class _Stage:
    def __init__(self, keep_removed_joint=False):
        self.prims = {
            "/World/TCP": _Prim(),
            "/World/Cube": _Prim(),
            "/World/Cube/sf_scripted_grasp_joint": _Prim(),
        }
        self.keep_removed_joint = keep_removed_joint
        self.removed_paths = []

    def GetPrimAtPath(self, path):
        return self.prims.get(str(path), _Prim(False))

    def Traverse(self):
        return tuple(self.prims.values())

    def RemovePrim(self, path):
        path = str(path)
        self.removed_paths.append(path)
        if not self.keep_removed_joint:
            self.prims.pop(path, None)


def _owned_stage_owner(keep_removed_joint=False):
    stage = _canonical_joint_stage(keep_removed_joint=keep_removed_joint)
    owner = UsdFixedJointOwner(stage, "/World/TCP")
    owner._owned_joint_path = "/World/Cube/sf_scripted_grasp_joint"
    owner._target_path = "/World/Cube"
    return owner, stage


def _canonical_joint_stage(keep_removed_joint=False, body0="/World/TCP"):
    stage = _Stage(keep_removed_joint=keep_removed_joint)
    stage.prims["/World/Cube/sf_scripted_grasp_joint"] = _Prim(
        type_name="PhysicsFixedJoint",
        path="/World/Cube/sf_scripted_grasp_joint",
        body0=body0,
        body1="/World/Cube",
    )
    return stage


def test_unowned_canonical_joint_is_reconciled_from_stage_signature():
    stage = _canonical_joint_stage()
    owner = UsdFixedJointOwner(stage, "/World/TCP")

    assert owner.reconcile_stale() is True
    assert stage.removed_paths == ["/World/Cube/sf_scripted_grasp_joint"]


def test_foreign_fixed_joint_is_preserved_during_reconciliation():
    stage = _canonical_joint_stage(body0="/World/ForeignTcp")
    owner = UsdFixedJointOwner(stage, "/World/TCP")

    assert owner.reconcile_stale() is False
    assert stage.removed_paths == []
    assert stage.GetPrimAtPath("/World/Cube/sf_scripted_grasp_joint").IsValid()


def test_reconciliation_failure_is_reported_without_clearing_state():
    stage = _canonical_joint_stage(keep_removed_joint=True)
    owner = UsdFixedJointOwner(stage, "/World/TCP")

    assert owner.reconcile_stale() is False
    assert stage.removed_paths == ["/World/Cube/sf_scripted_grasp_joint"]
    assert stage.GetPrimAtPath("/World/Cube/sf_scripted_grasp_joint").IsValid()


def test_usd_owner_clears_metadata_only_after_stage_confirms_joint_absence():
    owner, stage = _owned_stage_owner()

    assert owner.remove_owned() is True
    assert stage.removed_paths == ["/World/Cube/sf_scripted_grasp_joint"]
    assert owner.has_owned_joint() is False


def test_usd_owner_keeps_ownership_when_joint_remains_after_remove_request():
    owner, stage = _owned_stage_owner(keep_removed_joint=True)

    assert owner.remove_owned() is False
    assert stage.removed_paths == ["/World/Cube/sf_scripted_grasp_joint"]
    assert owner.has_owned_joint() is True
    assert owner.owned_joint_path == "/World/Cube/sf_scripted_grasp_joint"


def test_valid_candidate_creates_one_owned_joint_and_reports_held():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    assert len(joints.created) == 1
    assert instance.holding_observation().state is HoldingState.HELD
    assert instance.gripper.stop_calls == 0


@pytest.mark.parametrize(
    ("shape", "expected_reason"),
    [(None, "shape_tag_missing"), ("cylinder", "unsupported_shape")],
)
def test_invalid_or_unsupported_shape_fails_closed_after_close_completion(
    shape, expected_reason
):
    source = FakeCandidateSource([replace(valid_candidate(), shape=shape)])
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)
    instance.gripper.state.value = "holding"
    instance.arm_close_transaction()
    reasons = []
    mark_unknown = instance._mark_unknown

    def record_reason(reason):
        reasons.append(reason)
        return mark_unknown(reason)

    instance._mark_unknown = record_reason
    instance.update(0.01)

    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert expected_reason in reasons
    assert instance.gripper.stop_calls == 0


def test_unsupported_gripper_fails_after_close_without_attachment():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)
    instance.gripper.MODEL_ID = "other_gripper"
    instance.gripper.state.value = "holding"
    instance.arm_close_transaction()

    instance.update(0.01)

    assert instance.gripper.stop_calls == 0
    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_repeated_capture_then_open_release_preserves_held_and_released_cycles():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.HELD
    instance.note_open_release_command_applied(10, 85.0)
    instance.gripper.width_mm = 85.0
    instance.gripper.open()
    instance.gripper.finish_open()
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.RELEASED
    assert not joints.has_owned_joint()

    instance.gripper.state.value = "closing"
    assert instance.arm_close_transaction()
    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)

    assert len(joints.created) == 2
    assert instance.holding_observation().state is HoldingState.HELD


def test_holding_tick_allows_final_capture_assessment():
    source = FakeCandidateSource(
        [replace(valid_candidate(), tcp_local_position_m=(0.2, 0.0, 0.0))]
    )
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)
    instance.arm_close_transaction()
    instance.gripper.state.value = "closing"

    instance.update(0.01)
    assert joints.created == []
    assert instance._capture_transaction_armed is True

    source.candidates = [valid_candidate()]
    instance.gripper.state.value = "holding"
    instance.update(0.01)

    assert instance.gripper.stop_calls == 0
    assert len(joints.created) == 1
    assert instance.holding_observation().state is HoldingState.HELD


def test_close_in_progress_without_candidate_continues_search_without_terminal_state():
    source = FakeCandidateSource(
        [replace(valid_candidate(), tcp_local_position_m=(0.2, 0.0, 0.0))]
    )
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)
    instance.arm_close_transaction()
    instance.gripper.state.value = "closing"

    baseline = instance.holding_observation().sequence
    instance.update(0.01)
    instance.update(0.01)

    assert instance.holding_observation().sequence == baseline
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert instance._capture_transaction_armed is True
    assert source.query_calls == 2
    assert joints.created == []


def test_close_failure_does_not_query_or_capture_candidates():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.gripper.state.value = "idle"

    instance.update(0.01)

    assert source.query_calls == 0
    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert instance._capture_transaction_armed is False


def test_candidate_query_runtime_error_fails_closed_after_completed_close():
    source = FakeCandidateSource([valid_candidate()])

    def fail_query():
        raise RuntimeError("Stage query unavailable")

    source.query = fail_query
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert instance._capture_transaction_armed is False


def test_closing_search_captures_when_candidate_enters_volume_in_same_transaction():
    source = FakeCandidateSource(
        [replace(valid_candidate(), tcp_local_position_m=(0.2, 0.0, 0.0))]
    )
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)
    instance.arm_close_transaction()
    instance.gripper.state.value = "closing"

    create_joint = joints.create

    def create_after_stop(target_path):
        assert instance.gripper.state.value == "holding"
        return create_joint(target_path)

    joints.create = create_after_stop
    baseline = instance.holding_observation().sequence
    instance.update(0.01)
    assert joints.created == []
    assert source.query_calls == 1
    assert instance.holding_observation().sequence == baseline
    assert instance._capture_transaction_armed is True

    source.candidates = [valid_candidate()]
    instance.update(0.01)

    assert instance.gripper.stop_calls == 1
    observation = instance.holding_observation()
    assert observation.state is HoldingState.HELD
    assert len(joints.created) == 1
    assert instance._capture_transaction_armed is False

    instance.update(0.01)
    assert len(joints.created) == 1
    assert instance.holding_observation().sequence == observation.sequence


@pytest.mark.parametrize("failure", ["joint_creation", "ownership"])
def test_closing_capture_joint_failure_never_reports_held(failure):
    source = FakeCandidateSource([valid_candidate()])
    joints = (
        ExistingUnownedJointOwner() if failure == "joint_creation" else FakeJointOwner()
    )
    if failure == "ownership":
        joints.valid = False
    instance = manager(source, joints, arm=False)
    instance.arm_close_transaction()
    instance.gripper.state.value = "closing"

    instance.update(0.01)

    assert instance.gripper.stop_calls == 1
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert instance._capture_transaction_armed is False


def test_close_completion_without_candidate_fails_closed_after_final_evaluation():
    source = FakeCandidateSource(
        [replace(valid_candidate(), tcp_local_position_m=(0.2, 0.0, 0.0))]
    )
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)
    instance.arm_close_transaction()
    instance.gripper.state.value = "closing"

    instance.update(0.01)
    assert instance._capture_transaction_armed is True
    assert instance.holding_observation().state is HoldingState.UNKNOWN

    instance.gripper.state.value = "holding"
    instance.update(0.01)

    assert source.query_calls == 2
    assert joints.created == []
    assert instance._capture_transaction_armed is False
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_capture_attaches_inside_volume_despite_pose_yaw_and_width_diagnostics():
    capture_candidate = replace(
        valid_candidate(),
        position_error_m=0.5,
        orientation_error_deg=170.0,
        expected_orientation_error_deg=170.0,
        rawpart_orientation_error_deg=90.0,
        rawpart_rejection_reason="contact_width_tolerance_exceeded",
        measured_opening_mm=100.0,
    )
    joints = FakeJointOwner()
    instance = manager(FakeCandidateSource([capture_candidate]), joints)

    instance.update(0.01)

    observation = instance.holding_observation()
    assert len(joints.created) == 1
    assert observation.state is HoldingState.HELD
    assert observation.fresh()


def test_capture_record_is_goal_correlated_and_diagnostic_write_failure_is_noninterfering(
    tmp_path,
):
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    writer = BoundedCaptureDiagnosticWriter(tmp_path)
    instance.capture_reference_id = "goal-physx-1"
    instance.set_diagnostic_observers(writer=writer)
    writer._write_record = lambda _record: (_ for _ in ()).throw(OSError("disk full"))

    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.HELD
    assert instance.finalize_capture_diagnostic("focused_test") is True
    writer.close()

    assert writer.write_failure_count == 1
    assert instance.holding_observation().state is HoldingState.HELD
    assert joints.created == [("/World/Cube/sf_scripted_grasp_joint", "/World/Cube")]


def test_capture_writer_queue_saturation_does_not_change_holding_result(tmp_path):
    import threading

    entered = threading.Event()
    release = threading.Event()

    def slow_write(_record):
        entered.set()
        release.wait(2.0)

    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    writer = BoundedCaptureDiagnosticWriter(
        tmp_path, max_pending=1, write_record=slow_write
    )
    try:
        assert writer.submit({"id": "in_progress"})
        assert entered.wait(2.0)
        assert writer.submit({"id": "pending"})
        instance.capture_reference_id = "goal-writer-saturated"
        instance.set_diagnostic_observers(writer=writer)

        instance.update(0.01)
        assert instance.holding_observation().state is HoldingState.HELD
        assert instance.finalize_capture_diagnostic("queue_saturation") is False
        assert writer.dropped_count == 1
        assert instance.holding_observation().state is HoldingState.HELD
    finally:
        release.set()
        writer.close()


def test_capture_json_serialization_failure_does_not_change_holding_result(
    monkeypatch, tmp_path
):
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    writer = BoundedCaptureDiagnosticWriter(tmp_path)
    instance.capture_reference_id = "goal-json-failure"
    instance.set_diagnostic_observers(writer=writer)

    def fail_serialization(*_args, **_kwargs):
        raise TypeError("forced diagnostic serialization failure")

    monkeypatch.setattr(
        "gripper_runtime.capture_diagnostics.json.dumps", fail_serialization
    )
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.HELD
    assert instance.finalize_capture_diagnostic("serialization_failure") is True
    assert writer.close() is True
    assert writer.write_failure_count == 1
    assert instance.holding_observation().state is HoldingState.HELD


def test_fixed_joint_diagnostics_match_values_written_to_usd(monkeypatch):
    class _Attr:
        def __init__(self):
            self.value = None

        def Set(self, value):
            self.value = value

        def Get(self):
            return self.value

    class _Rel:
        def __init__(self):
            self.targets = []

        def SetTargets(self, values):
            self.targets = values

        def GetTargets(self):
            return self.targets

    class _JointPrim:
        def __init__(self):
            self.attrs = {}
            self.rels = {}

        def IsValid(self):
            return True

        def GetTypeName(self):
            return "PhysicsFixedJoint"

        def GetRelationship(self, name):
            return self.rels.setdefault(name, _Rel())

        def CreateBody0Rel(self):
            return self.GetRelationship("physics:body0")

        def CreateBody1Rel(self):
            return self.GetRelationship("physics:body1")

        def _attr(self, name):
            return self.attrs.setdefault(name, _Attr())

        def CreateLocalPos0Attr(self):
            return self._attr("physics:localPos0")

        def CreateLocalRot0Attr(self):
            return self._attr("physics:localRot0")

        def CreateLocalPos1Attr(self):
            return self._attr("physics:localPos1")

        def CreateLocalRot1Attr(self):
            return self._attr("physics:localRot1")

        def GetLocalPos0Attr(self):
            return self._attr("physics:localPos0")

        def GetLocalRot0Attr(self):
            return self._attr("physics:localRot0")

        def GetLocalPos1Attr(self):
            return self._attr("physics:localPos1")

        def GetLocalRot1Attr(self):
            return self._attr("physics:localRot1")

    class _Path(str):
        def AppendChild(self, name):
            return _Path(f"{self}/{name}")

    class _Stage:
        def __init__(self):
            self.prims = {"/World/TCP": _Prim(), "/World/Cube": _Prim()}

        def GetPrimAtPath(self, path):
            return self.prims.get(str(path), _Prim(False))

    stage = _Stage()
    holder = {}

    class _FixedJoint:
        @staticmethod
        def Define(_stage, path):
            joint = _JointPrim()
            _stage.prims[str(path)] = joint
            holder["joint"] = joint
            return joint

    class _Gf:
        Vec3f = staticmethod(lambda *values: tuple(values))
        Quatf = staticmethod(lambda real, imaginary: _Quaternion(real, imaginary))

    class _UsdPhysics:
        FixedJoint = _FixedJoint

    class _Sdf:
        Path = _Path

    monkeypatch.setattr(
        grasp_joint,
        "_pxr_modules",
        lambda: (_Gf, _Sdf, None, None, _UsdPhysics),
    )
    owner = UsdFixedJointOwner(stage, "/World/TCP")
    owner._world_transform = lambda path: _RigidWorldTransform(
        (1.0, 2.0, 3.0) if path == "/World/Cube" else (0.0, 0.0, 0.0),
        _Quaternion(),
    )

    assert owner.create("/World/Cube") is True
    detail = owner.last_create_diagnostics
    joint = holder["joint"]
    assert detail["joint_path"] == "/World/Cube/sf_scripted_grasp_joint"
    assert detail["body0_path"] == str(
        joint.GetRelationship("physics:body0").GetTargets()[0]
    )
    assert detail["body1_path"] == str(
        joint.GetRelationship("physics:body1").GetTargets()[0]
    )
    assert detail["local_pos0"] == joint.GetLocalPos0Attr().Get()
    assert detail["local_rot0"] == owner._quaternion_value(
        joint.GetLocalRot0Attr().Get()
    )
    assert detail["local_pos1"] == joint.GetLocalPos1Attr().Get()
    assert detail["local_rot1"] == owner._quaternion_value(
        joint.GetLocalRot1Attr().Get()
    )
    assert detail["stage_pose_before"] == detail["stage_pose_after"]


def test_terminal_closing_capture_error_publishes_fresh_unknown():
    source = FakeCandidateSource(
        [valid_candidate("/World/CubeA"), valid_candidate("/World/CubeB")]
    )
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    observation = instance.holding_observation()
    assert observation.sequence == 1
    assert observation.state is HoldingState.UNKNOWN
    assert joints.created == []


def test_unknown_runtime_during_closing_publishes_fresh_unknown():
    source = FakeCandidateSource([valid_candidate()])
    source.runtime_available = False
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    observation = instance.holding_observation()
    assert observation.sequence == 1
    assert observation.state is HoldingState.UNKNOWN
    assert joints.created == []


def test_closing_without_explicit_transaction_arm_does_not_capture():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)

    instance.update(0.01)

    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_terminal_close_without_capture_disarms_and_does_not_retry():
    source = FakeCandidateSource(
        [replace(valid_candidate(), tcp_local_position_m=(0.06, 0.0, 0.0))]
    )
    joints = FakeJointOwner()
    instance = manager(source, joints)

    baseline = instance.holding_observation().sequence
    instance.gripper.state.value = "closing"
    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert instance.holding_observation().sequence == baseline

    source.candidates = [valid_candidate()]
    instance.update(0.01)

    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_new_close_transaction_can_supersede_expired_close_failure():
    source = FakeCandidateSource([])
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.arm_close_transaction()
    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)

    source.candidates = [valid_candidate()]
    instance.gripper.state.value = "closing"
    instance.arm_close_transaction()
    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)

    assert len(joints.created) == 1
    assert instance.holding_observation().state is HoldingState.HELD


def test_terminal_failure_does_not_retry_while_closing_until_explicit_rearm():
    source = FakeCandidateSource([])
    joints = FakeJointOwner()
    instance = manager(source, joints, arm=False)
    instance.arm_close_transaction()

    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN

    instance.gripper.state.value = "closing"
    source.candidates = [valid_candidate()]
    instance.update(0.01)
    instance.update(0.01)
    assert joints.created == []

    instance.arm_close_transaction()
    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.update(0.01)

    assert len(joints.created) == 1
    assert instance.holding_observation().state is HoldingState.HELD


def test_capture_releases_stability_only_after_adapter_joint_creation():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    stability = FakePreCaptureStability(joints)
    instance = manager(source, joints, stability=stability)

    instance.update(0.01)

    assert stability.calls == [("/World/Cube", True)]
    assert instance.holding_observation().state is HoldingState.HELD


def test_stability_release_failure_removes_joint_and_never_reports_held():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    stability = FakePreCaptureStability(joints, result=False)
    instance = manager(source, joints, stability=stability)

    instance.update(0.01)

    assert joints.removed == ["/World/Cube/sf_scripted_grasp_joint"]
    assert not joints.has_owned_joint()
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert not instance.is_grasped
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_stability_release_cleanup_failure_fails_closed_without_held():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    joints.remove_result = False
    stability = FakePreCaptureStability(joints, result=False)
    instance = manager(source, joints, stability=stability)

    instance.update(0.01)

    assert joints.has_owned_joint()
    assert instance.ready is False
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert not instance.is_grasped


def test_applied_open_releases_after_capture_transition_failure():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    joints.valid = False
    instance = manager(source, joints)

    instance.update(0.01)

    assert instance._capture_transition_failed
    assert joints.has_owned_joint()
    joints.valid = True
    instance.gripper.state.value = "holding"
    instance.gripper.width_mm = 85.0
    instance.note_open_release_command_applied(21, 85.0)
    instance.update(0.01)

    assert not joints.has_owned_joint()
    assert instance.holding_observation().state is HoldingState.RELEASED
    assert not instance._capture_transaction_armed


def test_capture_transition_failure_without_open_keeps_attachment():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    joints.valid = False
    instance = manager(source, joints)

    instance.update(0.01)
    joints.valid = True
    instance.gripper.state.value = "holding"
    instance.gripper.width_mm = 85.0
    instance.update(0.01)

    assert joints.removed == []
    assert joints.has_owned_joint()
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert not instance._capture_transaction_armed


@pytest.mark.parametrize("failure", ["joint_state", "joint_remove"])
def test_capture_transition_failed_open_keeps_unknown_on_release_failure(failure):
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    joints.valid = False
    instance = manager(source, joints)
    instance.update(0.01)
    joints.valid = True
    instance.gripper.state.value = "holding"
    instance.gripper.width_mm = 85.0
    if failure == "joint_state":
        joints.valid = False
    else:
        joints.remove_result = False
    instance.note_open_release_command_applied(22, 85.0)

    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert not instance._capture_transaction_armed


def test_multiple_candidates_report_unknown_without_joint_creation():
    source = FakeCandidateSource(
        [valid_candidate("/World/CubeA"), valid_candidate("/World/CubeB")]
    )
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    assert joints.created == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_open_removes_only_owned_joint_and_reports_released():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)

    instance.gripper.finish_open()
    instance.release(open_gripper=False)

    assert len(joints.removed) == 1
    assert joints.foreign_paths == {"/World/ForeignJoint"}
    assert instance.holding_observation().state is HoldingState.RELEASED


def test_joint_remaining_after_remove_request_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    joints.remove_result = False

    instance.gripper.finish_open()
    instance.note_open_release_command_applied(1, 85.0)
    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert joints.has_owned_joint() is True


def test_missing_owned_joint_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    joints.valid = False

    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_unexpected_owned_joint_relation_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    joints.valid = False

    instance.gripper.finish_open()
    instance.note_open_release_command_applied(1, 85.0)
    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert joints.removed == []


def test_unavailable_stage_query_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    source.runtime_available = False
    joints = FakeJointOwner()
    instance = manager(source, joints)

    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_opening_transition_removes_owned_joint_and_reports_released():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)

    instance.gripper.open()
    instance.note_open_release_command_applied(1, 85.0)
    instance.gripper.finish_open()
    instance.update(0.01)

    assert len(joints.removed) == 1
    assert joints.foreign_paths == {"/World/ForeignJoint"}
    assert instance.holding_observation().state is HoldingState.RELEASED


def test_applied_open_detaches_when_actuator_is_already_at_open_target():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    instance.gripper.width_mm = 85.0
    instance.gripper.state.value = "holding"

    instance.note_open_release_command_applied(2, 85.0)
    instance.update(0.01)

    assert len(joints.removed) == 1
    assert instance.holding_observation().state is HoldingState.RELEASED


@pytest.mark.parametrize("width,state", [(12.0, "holding"), (85.0, "holding")])
def test_actuator_state_without_applied_open_never_detaches(width, state):
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    instance.gripper.width_mm = width
    instance.gripper.state.value = state

    instance.update(0.01)

    assert joints.removed == []
    assert instance.holding_observation().state is HoldingState.HELD


def test_duplicate_open_generation_cannot_repeat_release():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    instance.gripper.width_mm = 85.0
    instance.gripper.state.value = "holding"
    assert instance.note_open_release_command_applied(4, 85.0)
    instance.update(0.01)
    assert not instance.note_open_release_command_applied(4, 85.0)

    instance.update(0.01)

    assert len(joints.removed) == 1
    assert instance.holding_observation().state is HoldingState.RELEASED


def test_later_applied_non_open_command_cancels_pending_release():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    instance.note_open_release_command_applied(5, 85.0)
    instance.note_gripper_command_applied()
    instance.gripper.width_mm = 85.0
    instance.gripper.state.value = "holding"

    instance.update(0.01)

    assert joints.removed == []
    assert instance.holding_observation().state is HoldingState.HELD


def test_open_command_holding_below_target_does_not_detach_and_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)
    instance.gripper.state.value = "holding"
    instance.gripper.width_mm = 80.0
    instance.note_open_release_command_applied(6, 85.0)

    instance.update(0.01)

    assert joints.removed == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_target_disappearance_during_open_reports_unknown():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)

    joints.target_exists = False
    instance.gripper.finish_open()
    instance.note_open_release_command_applied(1, 85.0)
    instance.update(0.01)

    assert joints.removed == []
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_capture_query_unavailable_does_not_block_completed_open():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.update(0.01)

    source.runtime_available = False
    instance.gripper.finish_open()
    instance.note_open_release_command_applied(1, 85.0)
    instance.update(0.01)

    assert len(joints.removed) == 1
    assert instance.holding_observation().state is HoldingState.RELEASED


def test_missing_capture_configuration_rejects_close():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints, config=None)

    assert not instance.arm_close_transaction()
    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_invalid_capture_configuration_rejects_close():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    invalid = capture_config()
    invalid = CaptureConfig(capture_volume_dimensions_m=(0.0, 0.10, 0.10))
    instance = manager(source, joints, config=invalid)

    assert not instance.arm_close_transaction()
    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_valid_explicit_capture_configuration_marks_adapter_ready():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints, config=capture_config())

    assert instance.ready is True


def test_tilted_rawpart_allows_closing_axis_aligned_with_matching_side_face():
    angle = math.radians(30.0)
    tilted = _BasisTransform(
        x_axis=(0.012, 0.0, 0.0),
        y_axis=(0.0, 0.025 * math.cos(angle), 0.025 * math.sin(angle)),
        z_axis=(0.0, -0.030 * math.sin(angle), 0.030 * math.cos(angle)),
    )
    tcp = _BasisTransform()

    error = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp,
        tilted,
        expected_contact_width_mm=12.0,
        orientation_symmetry_order=2,
        orientation_tolerance_deg=5.0,
        contact_width_tolerance_mm=2.0,
    )

    assert error == pytest.approx(0.0)


def test_tilted_rawpart_rejects_closing_axis_that_cannot_contact_matching_faces():
    angle = math.radians(30.0)
    tilted = _BasisTransform(
        x_axis=(0.012, 0.0, 0.0),
        y_axis=(0.0, 0.025 * math.cos(angle), 0.025 * math.sin(angle)),
        z_axis=(0.0, -0.030 * math.sin(angle), 0.030 * math.cos(angle)),
    )
    tcp = _BasisTransform(x_axis=(0.0, 1.0, 0.0), y_axis=(-1.0, 0.0, 0.0))

    error = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp,
        tilted,
        expected_contact_width_mm=12.0,
        orientation_symmetry_order=2,
        orientation_tolerance_deg=5.0,
        contact_width_tolerance_mm=2.0,
    )

    assert error == pytest.approx(90.0)


def test_shape_width_mismatch_is_distinct_from_invalid_rawpart_geometry():
    width, reason = StageCaptureQuery._rawpart_orientation_error_deg(
        _BasisTransform(
            x_axis=(0.012, 0.0, 0.0),
            y_axis=(0.0, 0.025, 0.0),
            z_axis=(0.0, 0.0, 0.030),
        ),
        _BasisTransform(
            x_axis=(0.07, 0.0, 0.0),
            y_axis=(0.0, 0.08, 0.0),
            z_axis=(0.0, 0.0, 0.07),
        ),
        expected_contact_width_mm=18.0,
        orientation_symmetry_order=2,
        orientation_tolerance_deg=5.0,
        contact_width_tolerance_mm=2.0,
        return_reason=True,
    )

    assert width is None
    assert reason == "contact_width_tolerance_exceeded"


def test_capture_policy_keeps_rawpart_orientation_failure_diagnostic_only():
    candidate = replace(
        valid_candidate(),
        orientation_error_deg=float("inf"),
        rawpart_orientation_error_deg=None,
        rawpart_rejection_reason="rawpart_orientation_invalid",
    )

    result = CapturePolicy.evaluate(
        close_active=True,
        candidates=[candidate],
        config=capture_config(),
        runtime_available=True,
    )

    assert result.decision.value == "capture_eligible"
    assert result.candidate.prim_path == candidate.prim_path


def test_successful_capture_logs_one_user_facing_info(caplog, monkeypatch):
    instance = manager(FakeCandidateSource([valid_candidate()]), FakeJointOwner())
    instance.capture_reference_id = "motion-goal-success"
    monkeypatch.setattr(grasp_attachment.LOGGER, "propagate", True)
    caplog.set_level("INFO", logger="gripper_runtime.grasp_attachment")

    instance.update(1.0 / 60.0)
    assert instance.holding_observation().state is HoldingState.HELD

    messages = [
        record.message
        for record in caplog.records
        if "Gripper captured an object" in record.message
    ]
    assert len(messages) == 1
    assert messages == ["Gripper captured an object and is holding it"]


def test_fixed_joint_create_returning_false_has_create_failure_reason(
    caplog, monkeypatch
):
    instance = manager(
        FakeCandidateSource([valid_candidate()]), ExistingUnownedJointOwner()
    )
    monkeypatch.setattr(grasp_attachment.LOGGER, "propagate", True)
    caplog.set_level("WARNING", logger="gripper_runtime.grasp_attachment")

    instance.update(1.0 / 60.0)

    warning = next(
        record.message
        for record in caplog.records
        if "Gripper could not attach" in record.message
    )
    assert "attachment_create_failed" in warning
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_capture_diagnostic_writer_failure_does_not_change_held_result():
    class _Writer:
        def submit(self, _record):
            raise OSError("diagnostic store unavailable")

    instance = manager(FakeCandidateSource([valid_candidate()]), FakeJointOwner())
    instance.capture_reference_id = "motion-goal-writer-failure"
    instance.set_diagnostic_observers(writer=_Writer())

    instance.update(1.0 / 60.0)
    instance.finalize_capture_diagnostic("unit-test-finalize")

    assert instance.holding_observation().state is HoldingState.HELD
    assert instance.joint_owner.has_owned_joint() is True


def test_reference_invalidation_does_not_reuse_capture_transition_reason():
    class _Writer:
        def __init__(self):
            self.records = []

        def submit(self, record):
            self.records.append(record)
            return True

    writer = _Writer()
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    instance.capture_reference_id = "motion-goal-close-transition"
    instance.set_diagnostic_observers(writer=writer)

    instance._record(HoldingState.RELEASED, "pre_close_release")
    assert instance.arm_close_transaction("motion-goal-close-transition", 17)
    instance._mark_unknown("capture_rejected")
    instance._diagnostic_outcome(
        "capture_reference_invalidated",
        capture_decision="no_capture",
        capture_result="rejected",
    )

    record = writer.records[0]
    assert record["motion_goal_uuid"] == "motion-goal-close-transition"
    arm = next(
        event
        for event in record["events"]
        if event["payload"].get("phase") == "close_transaction_arm"
    )
    assert arm["payload"]["command_generation"] == 17
    transition = next(
        event
        for event in record["events"]
        if event["payload"].get("phase") == "holding_transition"
    )
    assert transition["payload"]["reason"] == "capture_rejected"
    outcome = record["events"][-1]["payload"]
    assert outcome["reason"] == "capture_reference_invalidated"
    assert "holding_transition_reason" not in outcome


def test_consecutive_capture_failures_do_not_reuse_transition_cause():
    class _Writer:
        def __init__(self):
            self.records = []

        def submit(self, record):
            self.records.append(record)
            return True

    writer = _Writer()
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    instance.capture_reference_id = "motion-goal-stale-transition"
    instance.set_diagnostic_observers(writer=writer)
    instance._record(HoldingState.RELEASED, "prior_release")
    instance._ensure_capture_diagnostic_record(instance.capture_reference_id)
    transitioned = instance._mark_unknown("first_capture_rejected")
    instance._diagnostic_outcome(
        "first_capture_rejected",
        capture_result="rejected",
        transition_reason="first_capture_rejected" if transitioned else None,
    )
    instance._ensure_capture_diagnostic_record(instance.capture_reference_id)
    transitioned = instance._mark_unknown("second_capture_rejected")
    instance._diagnostic_outcome(
        "second_capture_rejected",
        capture_result="rejected",
        transition_reason="second_capture_rejected" if transitioned else None,
    )

    first = writer.records[0]["events"][-1]["payload"]
    second = writer.records[1]["events"][-1]["payload"]
    assert first["reason"] == "first_capture_rejected"
    assert first["holding_transition_reason"] == "first_capture_rejected"
    assert second["reason"] == "second_capture_rejected"
    assert "holding_transition_reason" not in second


def test_new_motion_goal_finalizes_previous_capture_without_transition_cause():
    class _Writer:
        def __init__(self):
            self.records = []

        def submit(self, record):
            self.records.append(record)
            return True

    writer = _Writer()
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    instance.set_diagnostic_observers(writer=writer)
    instance.capture_reference_id = "previous-motion-goal"
    instance._ensure_capture_diagnostic_record(instance.capture_reference_id)
    instance._record(HoldingState.HELD, "previous_capture_held")

    next_record = instance._ensure_capture_diagnostic_record("next-motion-goal")

    previous_outcome = writer.records[0]["events"][-1]["payload"]
    assert previous_outcome["reason"] == "capture_reference_replaced"
    assert "holding_transition_reason" not in previous_outcome
    assert next_record.motion_goal_uuid == "next-motion-goal"


@pytest.mark.parametrize("order", [0, 1, 2, 4])
def test_motion_reference_accepts_supported_order_with_selected_concrete_pose(order):
    query = StageCaptureQuery(None, "/robot/tcp", EligibleObjectRegistry(), None)
    assert query.configure_expected_tcp_reference(
        "base_link", (0.2, 0.1, 0.3), (0.0, 1.0, 0.0, 0.0), order, 70.0
    )
    assert query.expected_tcp_reference == (
        "base_link",
        (0.2, 0.1, 0.3),
        (0.0, 1.0, 0.0, 0.0),
        order,
        70.0,
    )
    query.clear_expected_tcp_reference()

    class _Prim:
        def IsValid(self):
            return True

    query.stage = SimpleNamespace(GetPrimAtPath=lambda _path: _Prim())
    query.registry = SimpleNamespace(candidate_paths=lambda _stage: [])
    query._world_transform = lambda _path: object()
    query._pose_dict = lambda _transform: {}
    assert query.query() == (True, [])


@pytest.mark.parametrize("order", [0, 1])
def test_validation_reference_checks_side_face_geometry_without_top_down_gate(order):
    tcp = _BasisTransform((0, 0, 1), (0, 1, 0), (-1, 0, 0))
    box = _BasisTransform((0.07, 0, 0), (0, 0.07, 0), (0, 0, 0.08))
    assert (
        StageCaptureQuery._rawpart_orientation_error_deg(tcp, box, 80, order, 10, 2)
        == 0
    )
    error, reason = StageCaptureQuery._rawpart_orientation_error_deg(
        tcp, box, 70, order, 10, 2, return_reason=True
    )
    assert error > 10
    assert reason is None


@pytest.mark.parametrize("path", ["pending", "update", "direct"])
def test_completed_open_reports_released_without_capture_reference(path):
    source = StageCaptureQuery(None, "/World/TCP", None, None)
    instance = manager(source, FakeJointOwner(), arm=False)
    instance.gripper.state.value = "holding"
    instance.gripper.width_mm = 85.0
    if path == "pending":
        instance.note_open_release_command_applied(1, 85.0)
    if path == "direct":
        instance.release(open_gripper=False)
    else:
        instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.RELEASED
    assert source.expected_tcp_reference is None
    assert source.reference_diagnostics() is None


def test_released_observation_ignores_idle_open_width_jitter_without_attachment():
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    instance.gripper.finish_open()
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.RELEASED

    instance.gripper.width_mm = 84.9
    instance.update(0.01)

    assert instance.holding_observation().state is HoldingState.RELEASED


@pytest.mark.parametrize(
    "valid,expected", [(True, HoldingState.HELD), (False, HoldingState.UNKNOWN)]
)
def test_released_observation_still_checks_owned_attachment(valid, expected):
    joints = FakeJointOwner()
    instance = manager(FakeCandidateSource([]), joints, arm=False)
    instance.gripper.finish_open()
    instance.update(0.01)
    joints.owned_path = "/World/Target/sf_scripted_grasp_joint"
    joints.valid = valid

    instance.gripper.width_mm = 84.9
    instance.update(0.01)

    assert instance.holding_observation().state is expected


@pytest.mark.parametrize("path", ["pending", "update", "direct"])
@pytest.mark.parametrize(
    "state,width", [("opening", 85.0), ("holding", 80.0), ("idle", 85.0)]
)
def test_release_paths_require_completed_physical_open(path, state, width):
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    instance.gripper.state.value = state
    instance.gripper.width_mm = width
    if path == "pending":
        instance.note_open_release_command_applied(1, 85.0)
    if path == "direct":
        instance.release(open_gripper=False)
    else:
        instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_pending_open_keeps_joint_until_actuator_completion():
    joints = FakeJointOwner()
    instance = manager(FakeCandidateSource([valid_candidate()]), joints)
    instance.update(0.01)
    instance.gripper.open()
    instance.note_open_release_command_applied(1, 85.0)
    instance.gripper.width_mm = 85.0
    instance.update(0.01)
    assert joints.has_owned_joint()
    assert instance.holding_observation().state is HoldingState.HELD
    instance.gripper.state.value = "holding"
    instance.update(0.01)
    assert not joints.has_owned_joint()
    assert instance.holding_observation().state is HoldingState.RELEASED


def test_direct_release_waits_for_actuator_completion_then_detaches():
    joints = FakeJointOwner()
    instance = manager(FakeCandidateSource([valid_candidate()]), joints)
    instance.update(0.01)
    instance.release()
    assert joints.has_owned_joint()
    assert instance.holding_observation().state is HoldingState.HELD
    instance.gripper.width_mm = 85.0
    instance.gripper.state.value = "holding"
    instance.update(0.01)
    assert not joints.has_owned_joint()
    assert instance.holding_observation().state is HoldingState.RELEASED


@pytest.mark.parametrize("path", ["pending", "update", "direct"])
def test_unreadable_joint_state_remains_unknown(path):
    joints = FakeJointOwner()
    instance = manager(FakeCandidateSource([]), joints, arm=False)
    instance.gripper.state.value = "holding"
    instance.gripper.width_mm = 85.0

    def unavailable():
        raise RuntimeError("joint state unavailable")

    joints.has_owned_joint = unavailable
    if path == "pending":
        instance.note_open_release_command_applied(1, 85.0)
    if path == "direct":
        instance.release(open_gripper=False)
    else:
        instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_close_without_motion_reference_can_attach_by_capture_volume():
    source = FakeCandidateSource([valid_candidate()])
    joints = FakeJointOwner()
    instance = manager(source, joints)
    instance.gripper.state.value = "holding"
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.HELD
    assert joints.has_owned_joint()


@pytest.mark.parametrize(
    "failure,expected",
    [
        ("width_nan", "actuator_width_unavailable"),
        ("width_read", "actuator_width_unavailable"),
        ("below_target", "open_target_not_reached"),
        ("joint_read", "joint_state_unavailable"),
        ("joint_unknown", "joint_state_unavailable"),
        ("joint_invalid", "joint_state_unavailable"),
        ("remove_false", "joint_remove_failed"),
        ("remove_throws", "joint_remove_failed"),
        ("joint_remains", "joint_remove_failed"),
    ],
)
def test_release_diagnostic_distinguishes_failed_physical_condition(failure, expected):
    joints = FakeJointOwner()
    instance = manager(FakeCandidateSource([valid_candidate()]), joints)
    instance.update(0.01)
    instance.gripper.finish_open()

    def unavailable():
        raise RuntimeError("observation unavailable")

    if failure == "width_nan":
        instance.gripper.width_mm = float("nan")
    elif failure == "width_read":
        instance.gripper.get_width = unavailable
    elif failure == "below_target":
        instance.gripper.width_mm = 80.0
    elif failure == "joint_read":
        joints.has_owned_joint = unavailable
    elif failure == "joint_unknown":
        joints.has_owned_joint = lambda: None
    elif failure == "joint_invalid":
        joints.valid = False
    elif failure == "remove_false":
        joints.remove_result = False
    elif failure == "remove_throws":
        joints.remove_owned = unavailable
    elif failure == "joint_remains":
        joints.remove_owned = lambda: True
    instance.note_open_release_command_applied(1, 85.0)
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert instance.consume_release_detach_event()["result"] == expected
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN


@pytest.mark.parametrize("initial", ["unknown", "held", "released"])
def test_pending_open_publishes_new_sequence_only_at_physical_outcome(initial):
    joints = FakeJointOwner()
    instance = manager(
        FakeCandidateSource([valid_candidate()]), joints, arm=initial == "held"
    )
    if initial == "held":
        instance.update(0.01)
    elif initial == "released":
        instance.gripper.finish_open()
        instance.update(0.01)
    baseline = instance.holding_observation()
    instance.gripper.open()
    instance.note_open_release_command_applied(1, 85.0)
    instance.update(0.01)
    pending = instance.holding_observation()
    assert pending.sequence == baseline.sequence
    assert pending.state == baseline.state
    instance.gripper.finish_open()
    instance.update(0.01)
    completed = instance.holding_observation()
    assert completed.state is HoldingState.RELEASED
    assert completed.sequence > baseline.sequence


@pytest.mark.parametrize("path", ["pending", "update", "direct"])
def test_capture_configuration_is_not_a_physical_release_condition(path):
    joints = FakeJointOwner()
    instance = manager(FakeCandidateSource([]), joints, config=None, arm=False)
    instance.gripper.finish_open()
    if path == "pending":
        instance.note_open_release_command_applied(1, 85.0)
    if path == "direct":
        instance.release(open_gripper=False)
    else:
        instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.RELEASED
    assert not instance.arm_close_transaction()


@pytest.mark.parametrize("config", [None, CaptureConfig((0.0, 0.1, 0.1))])
def test_capture_configuration_failure_cannot_turn_into_release_without_open(config):
    joints = FakeJointOwner()
    instance = manager(
        FakeCandidateSource([valid_candidate()]), joints, config=config, arm=False
    )
    instance.gripper.finish_open()
    assert not instance.arm_close_transaction()
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN
    assert not joints.has_owned_joint()
    instance.note_open_release_command_applied(1, 85.0)
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.RELEASED


@pytest.mark.parametrize("path", ["pending", "update", "direct"])
def test_unavailable_lifecycle_does_not_publish_release(path):
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    instance.gripper.finish_open()
    instance.ready = False
    if path == "pending":
        instance.note_open_release_command_applied(1, 85.0)
    if path == "direct":
        instance.release(open_gripper=False)
    else:
        instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN


def test_direct_open_runtime_error_remains_unknown_without_new_open():
    instance = manager(FakeCandidateSource([]), FakeJointOwner(), arm=False)
    instance.gripper.finish_open()

    def fail_open(**_kwargs):
        raise RuntimeError("actuator command failed")

    instance.gripper.open = fail_open
    instance.release()
    assert instance.consume_release_detach_event()["result"] == "runtime_error"
    instance.update(0.01)
    assert instance.holding_observation().state is HoldingState.UNKNOWN
