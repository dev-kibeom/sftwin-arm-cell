#include "arm_cell_vda_adapter/external_state_mock.hpp"

namespace arm_cell_vda_adapter
{

void ExternalStateMock::set_fault(Fault fault, bool active)
{
  switch (fault) {
    case Fault::kEStop:
      e_stop_ = active;
      break;
    case Fault::kPrematureUndock:
      if (active) {
        amr_docking_state_ = AMRDockingState::AMR_DOCKING_UNDOCKED;
        amr_driving_ = true;
      }
      break;
    case Fault::kPackmlAbort:
      packml_abort_ = active;
      break;
    case Fault::kInvalidState:
      invalid_state_ = active;
      break;
    case Fault::kCommunicationLoss:
      communication_loss_ = active;
      break;
    case Fault::kCommunicationDegradation:
      communication_degradation_ = active;
      break;
    case Fault::kMaterialHandoffFailure:
      break;
  }
}

ExternalStateSnapshot ExternalStateMock::snapshot(const rclcpp::Time & stamp) const
{
  ExternalStateSnapshot result;
  result.publish = !communication_loss_;
  result.communication_degraded = communication_degradation_;

  result.amr.header.stamp = stamp;
  result.amr.amr_id = "mock_amr";
  result.amr.current_node_id = "arm_cell_dock";
  result.amr.docking_state = amr_docking_state_;
  result.amr.driving = amr_driving_;
  result.amr.valid = true;

  result.packml.header.stamp = stamp;
  result.packml.state = packml_abort_ ?
    PackMLState::PACKML_STATE_ABORTED : PackMLState::PACKML_STATE_IDLE;
  result.packml.valid = true;

  result.hardware.header.stamp = stamp;
  result.hardware.e_stop_active = e_stop_;
  result.hardware.sto_active = false;
  result.hardware.valid = true;

  if (invalid_state_) {
    result.amr.docking_state = AMRDockingState::AMR_DOCKING_UNKNOWN;
    result.amr.driving = false;
    result.amr.valid = false;
    result.packml.state = PackMLState::PACKML_STATE_UNKNOWN;
    result.packml.valid = false;
    result.hardware.e_stop_active = false;
    result.hardware.sto_active = false;
    result.hardware.valid = false;
  }

  return result;
}

}  // namespace arm_cell_vda_adapter
