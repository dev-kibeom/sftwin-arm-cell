#pragma once

#include <cstdint>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <arm_cell_interfaces/msg/amr_docking_state.hpp>
#include <arm_cell_interfaces/msg/pack_ml_state.hpp>
#include <arm_cell_interfaces/msg/safety_hardware_state.hpp>

namespace arm_cell_vda_adapter
{

using AMRDockingState = arm_cell_interfaces::msg::AMRDockingState;
using PackMLState = arm_cell_interfaces::msg::PackMLState;
using SafetyHardwareState = arm_cell_interfaces::msg::SafetyHardwareState;

// These faults are mock-only SetBool controls under ~/fault/*; they are not
// canonical ARM Cell control interfaces and do not reset Safety or stop Motion.
enum class Fault : std::uint8_t
{
  kEStop,
  kPrematureUndock,
  kPackmlAbort,
  kInvalidState,
  kCommunicationLoss,
  kCommunicationDegradation,
  kMaterialHandoffFailure,
};

struct ExternalStateSnapshot
{
  bool publish{true};
  bool communication_degraded{false};
  AMRDockingState amr;
  PackMLState packml;
  SafetyHardwareState hardware;
};

class ExternalStateMock final
{
public:
  void set_fault(Fault fault, bool active);

  [[nodiscard]] ExternalStateSnapshot snapshot(const rclcpp::Time & stamp) const;

private:
  bool e_stop_{false};
  std::uint8_t amr_docking_state_{AMRDockingState::AMR_DOCKING_DOCKED};
  bool amr_driving_{false};
  bool packml_abort_{false};
  bool invalid_state_{false};
  bool communication_loss_{false};
  bool communication_degradation_{false};
};

}  // namespace arm_cell_vda_adapter
