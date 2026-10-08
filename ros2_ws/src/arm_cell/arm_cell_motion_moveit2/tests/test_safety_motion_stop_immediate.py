"""Verify SafetyNode dispatches PackML abort through the real stop service."""

import importlib.util
from pathlib import Path

from arm_cell_interfaces.msg import AMRDockingState, MotionStatus, PackMLState, StopMode

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


class TestSafetyMotionStopImmediate(TestSafetyMotionStopIntegration):
    test_safety_dispatches_and_observes_confirmed_direct_stop = None

    def test_packml_abort_dispatches_immediate_stop_without_orchestration(self):
        """VR-ICD-SAFE-MOT-04/05: Safety selects and dispatches IMMEDIATE."""
        self.spin_until(self.action_client.server_is_ready)
        self.establish_safe_state()
        self.goal_handle = self.send_goal_after_safe_state()
        self.spin_until(
            lambda: self.latest_motion_state(MotionStatus.MOTION_STATE_EXECUTING)
        )

        self.publish_inputs(
            AMRDockingState.AMR_DOCKING_DOCKED,
            PackMLState.PACKML_STATE_ABORTED,
        )
        self.spin_until(
            lambda: any(
                mode.value == StopMode.STOP_MODE_IMMEDIATE for mode in self.stop_modes
            )
        )
        self.assertTrue(
            any(
                state.selected_stop_mode.value == StopMode.STOP_MODE_IMMEDIATE
                for state in self.safety_states
            )
        )
        observed_modes = [mode.value for mode in self.stop_modes]
        self.assertIn(StopMode.STOP_MODE_IMMEDIATE, observed_modes)
        self.assertIn(StopMode.STOP_MODE_CONTROLLED, observed_modes)
