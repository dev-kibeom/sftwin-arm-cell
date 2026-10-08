"""Verify inactive Motion does not receive a redundant direct stop."""

import importlib.util
from pathlib import Path

import rclpy

from arm_cell_interfaces.msg import AMRDockingState, PackMLState, SafetyState

_BASE_SPEC = importlib.util.spec_from_file_location(
    "safety_motion_stop_base",
    Path(__file__).with_name("test_safety_motion_stop_integration.py"),
)
_BASE = importlib.util.module_from_spec(_BASE_SPEC)
_BASE_SPEC.loader.exec_module(_BASE)
TestSafetyMotionStopIntegration = _BASE.TestSafetyMotionStopIntegration
_generate_test_description = _BASE.generate_test_description
TestSafetyMotionStopIntegration.__test__ = False
TestSafetyMotionStopIntegration.test_safety_dispatches_and_observes_confirmed_direct_stop = (
    None
)


def generate_test_description():
    return _generate_test_description()


class TestSafetyMotionStopNegative(TestSafetyMotionStopIntegration):
    test_safety_dispatches_and_observes_confirmed_direct_stop = None

    def test_inactive_motion_does_not_dispatch_redundant_direct_stop(self):
        """Negative path: unsafe input with inactive Motion sends no stop."""
        self.spin_until(self.action_client.server_is_ready)
        self.establish_safe_state()
        self.publish_inputs(
            AMRDockingState.AMR_DOCKING_UNDOCKED,
            PackMLState.PACKML_STATE_EXECUTE,
        )
        self.spin_until(
            lambda: self.latest_safety_state(SafetyState.SAFETY_STATE_INTERLOCKED)
        )
        for _ in range(15):
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(self.stop_modes, [])
