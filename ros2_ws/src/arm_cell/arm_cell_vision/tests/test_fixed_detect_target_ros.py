"""ROS-facing contract tests for the validation-only Fixed Vision node."""

import os
import signal
import subprocess
import time

import pytest
import rclpy
from arm_cell_interfaces.msg import DetectTargetResultCode
from arm_cell_interfaces.srv import DetectTarget
from arm_cell_vision.srv import RegisterFixedTarget
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def _pose(frame_id="base_link", x=0.2, y=-0.1, z=0.35, stamp=(7, 123456789)):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp.sec = stamp[0]
    pose.header.stamp.nanosec = stamp[1]
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    return pose


def _registration(
    target_id="cube_profile",
    run_id="run-1",
    pose=None,
    has_target_yaw=True,
    ttl=(0, 0),
):
    request = RegisterFixedTarget.Request()
    request.target_id = target_id
    request.run_id = run_id
    request.pose = pose or _pose()
    request.has_target_yaw = has_target_yaw
    request.ttl.sec = ttl[0]
    request.ttl.nanosec = ttl[1]
    return request


def _detect(target_id="cube_profile"):
    request = DetectTarget.Request()
    request.target_id = target_id
    request.timeout.sec = 1
    return request


def _spin_until(node, future, timeout=3.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done(), "ROS service request did not complete"
    return future.result()


class FixedVisionRosHarness:
    def __init__(self, domain_id):
        self.environment = os.environ.copy()
        self.environment["ROS_DOMAIN_ID"] = domain_id
        self.node = Node("fixed_vision_ros_contract_test")
        self.registration_client = self.node.create_client(
            RegisterFixedTarget,
            "/fixed_detect_target_node/register_target",
        )
        self.detect_client = self.node.create_client(
            DetectTarget,
            "/vision/detect_target",
        )
        self.process = subprocess.Popen(
            [
                "ros2",
                "run",
                "arm_cell_vision",
                "fixed_detect_target_node",
                "--ros-args",
                "-p",
                "target_id:=cube_profile",
            ],
            env=self.environment,
            start_new_session=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self._wait_for_services()

    def _wait_for_services(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                stdout, stderr = self.process.communicate(timeout=1)
                raise AssertionError(
                    "fixed_detect_target_node exited before advertising services: "
                    f"returncode={self.process.returncode}, "
                    f"stdout={stdout!r}, stderr={stderr!r}"
                )
            if (
                self.registration_client.wait_for_service(timeout_sec=0.2)
                and self.detect_client.wait_for_service(timeout_sec=0.2)
            ):
                return
        raise AssertionError("Fixed Vision ROS services did not become available")

    def register(self, request):
        return _spin_until(self.node, self.registration_client.call_async(request))

    def detect(self, request):
        return _spin_until(self.node, self.detect_client.call_async(request))

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
        self.process.communicate(timeout=1)
        self.node.destroy_node()


@pytest.fixture(scope="module")
def fixed_vision():
    domain_id = str(100 + (os.getpid() + int(time.monotonic() * 1000)) % 100)
    os.environ["ROS_DOMAIN_ID"] = domain_id
    rclpy.init()
    harness = FixedVisionRosHarness(domain_id)
    try:
        yield harness
    finally:
        harness.close()
        rclpy.shutdown()


def test_registration_detects_registered_pose_and_consumes_once(fixed_vision):
    response = fixed_vision.register(
        _registration(has_target_yaw=True, pose=_pose(stamp=(11, 22)))
    )
    assert response.accepted
    assert response.receipt_sequence > 0
    assert response.receipt_time_ns > 0

    result = fixed_vision.detect(_detect())
    assert result.result_code.value == DetectTargetResultCode.DETECT_RESULT_SUCCESS
    assert result.has_target_pose
    assert result.has_target_yaw
    assert result.target_pose.header.frame_id == "base_link"
    assert result.target_pose.header.stamp.sec == 11
    assert result.target_pose.header.stamp.nanosec == 22
    assert result.target_pose.pose.position.x == pytest.approx(0.2)
    assert result.target_pose.pose.position.y == pytest.approx(-0.1)
    assert result.target_pose.pose.position.z == pytest.approx(0.35)
    assert result.target_pose.pose.orientation.w == pytest.approx(1.0)

    depleted = fixed_vision.detect(_detect())
    assert (
        depleted.result_code.value
        == DetectTargetResultCode.DETECT_RESULT_OBJECT_NOT_FOUND
    )


def test_unknown_target_returns_invalid_result(fixed_vision):
    result = fixed_vision.detect(_detect("unknown_profile"))
    assert (
        result.result_code.value
        == DetectTargetResultCode.DETECT_RESULT_INVALID_RESULT
    )


@pytest.mark.parametrize(
    "registration_request",
    [
        _registration(pose=_pose(frame_id="camera_link")),
        _registration(pose=_pose(x=float("nan"))),
        _registration(pose=PoseStamped()),
        _registration(run_id=""),
        _registration(ttl=(61, 0)),
    ],
    ids=["wrong-frame", "non-finite", "zero-norm", "empty-run", "ttl-over-cap"],
)
def test_registration_rejects_invalid_requests(fixed_vision, registration_request):
    response = fixed_vision.register(registration_request)
    assert not response.accepted
    assert response.receipt_sequence == 0


def test_different_run_isolated_and_same_run_replaces_record(fixed_vision):
    first = fixed_vision.register(
        _registration(run_id="run-a", pose=_pose(x=0.1))
    )
    assert first.accepted

    different_run = fixed_vision.register(
        _registration(run_id="run-b", pose=_pose(x=0.9))
    )
    assert not different_run.accepted

    replacement = fixed_vision.register(
        _registration(run_id="run-a", pose=_pose(x=0.7), has_target_yaw=False)
    )
    assert replacement.accepted

    result = fixed_vision.detect(_detect())
    assert result.result_code.value == DetectTargetResultCode.DETECT_RESULT_SUCCESS
    assert result.target_pose.pose.position.x == pytest.approx(0.7)
    assert not result.has_target_yaw


def test_zero_ttl_is_accepted_as_default_and_short_ttl_expires(fixed_vision):
    default_ttl = fixed_vision.register(_registration(run_id="default-ttl", ttl=(0, 0)))
    assert default_ttl.accepted
    assert fixed_vision.detect(_detect()).result_code.value == (
        DetectTargetResultCode.DETECT_RESULT_SUCCESS
    )

    short_ttl = fixed_vision.register(
        _registration(run_id="expiring", ttl=(0, 1))
    )
    assert short_ttl.accepted
    time.sleep(0.05)
    expired = fixed_vision.detect(_detect())
    assert (
        expired.result_code.value
        == DetectTargetResultCode.DETECT_RESULT_OBJECT_NOT_FOUND
    )


def test_false_yaw_preserves_registration_provenance_without_tool_correction(fixed_vision):
    pose = _pose(x=0.31, y=0.12, z=0.44, stamp=(19, 987654321))
    response = fixed_vision.register(
        _registration(
            run_id="no-yaw",
            pose=pose,
            has_target_yaw=False,
        )
    )
    assert response.accepted

    result = fixed_vision.detect(_detect())
    assert result.result_code.value == DetectTargetResultCode.DETECT_RESULT_SUCCESS
    assert not result.has_target_yaw
    assert result.target_pose.header.frame_id == "base_link"
    assert result.target_pose.header.stamp.sec == 19
    assert result.target_pose.header.stamp.nanosec == 987654321
    assert result.target_pose.pose.position.x == pytest.approx(0.31)
    assert result.target_pose.pose.position.y == pytest.approx(0.12)
    assert result.target_pose.pose.position.z == pytest.approx(0.44)
