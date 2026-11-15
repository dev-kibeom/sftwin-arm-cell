#include "arm_cell_motion/isaac_gripper_port.hpp"

#include <cmath>

#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

namespace arm_cell_motion
{

IsaacGripperPort::IsaacGripperPort(std::shared_ptr<IsaacMotionTransport> transport)
: transport_(std::move(transport))
{
}

uint8_t IsaacGripperPort::close()
{
  if (!transport_) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  if (!transport_->command_gripper_close())
  {
    return transport_->available() ?
           arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE :
           arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

uint8_t IsaacGripperPort::close(double grip_force_n)
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  if (!std::isfinite(grip_force_n) || grip_force_n <= 0.0) {
    return Code::TASK_RESULT_INVALID_GOAL;
  }
  if (!transport_) {return Code::TASK_RESULT_BACKEND_UNAVAILABLE;}
  if (!transport_->command_gripper_close(grip_force_n)) {
    return transport_->available() ? Code::TASK_RESULT_GRASP_FAILURE :
           Code::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  return Code::TASK_RESULT_SUCCESS;
}

uint8_t IsaacGripperPort::open()
{
  if (!transport_) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  return transport_->command_gripper_release() ?
         arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS :
         arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
}

HoldingObservation IsaacGripperPort::holding() const
{
  if (!transport_) {
    return {};
  }
  return transport_->holding_observation();
}

bool IsaacGripperPort::active() const
{
  return transport_ && transport_->gripper_active();
}

void IsaacGripperPort::stop()
{
  if (transport_) {
    transport_->stop_gripper();
  }
}

}  // namespace arm_cell_motion
