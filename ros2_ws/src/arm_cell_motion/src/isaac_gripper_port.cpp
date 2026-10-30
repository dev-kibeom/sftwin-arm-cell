#include "arm_cell_motion/isaac_gripper_port.hpp"
#include "arm_cell_motion/isaac_ros_motion_transport.hpp"

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

uint8_t IsaacGripperPort::open()
{
  if (!transport_) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  return transport_->command_normalized_gripper(1.0) ?
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
