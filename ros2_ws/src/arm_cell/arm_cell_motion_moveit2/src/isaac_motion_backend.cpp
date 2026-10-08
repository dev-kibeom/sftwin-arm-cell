#include "arm_cell_motion_moveit2/isaac_motion_backend.hpp"

#include "arm_cell_motion_moveit2/isaac_gripper_port.hpp"

#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

namespace arm_cell_motion_moveit2
{

namespace
{
constexpr float kFullyOpenWidthMm = 85.0F;
}

IsaacMotionBackend::IsaacMotionBackend(std::shared_ptr<IsaacMotionTransport> transport)
: transport_(std::move(transport))
{
  gripper_port_ = std::make_shared<IsaacGripperPort>(transport_);
}

std::shared_ptr<GripperPort> IsaacMotionBackend::gripper_port() const
{
  return gripper_port_;
}

bool IsaacMotionBackend::available() const
{
  return transport_ && transport_->available();
}

bool IsaacMotionBackend::apply_runtime_tuning(const MotionTuning & tuning)
{
  return transport_ && transport_->apply_runtime_tuning(tuning);
}

bool IsaacMotionBackend::set_motion_envelope(const MotionEnvelope & envelope)
{
  return transport_ && transport_->set_motion_envelope(envelope);
}

MotionEnvelope IsaacMotionBackend::applied_motion_envelope() const
{
  return transport_ ? transport_->applied_motion_envelope() : MotionEnvelope{};
}

std::string IsaacMotionBackend::availability_diagnostic() const
{
  return transport_ ? transport_->availability_diagnostic() :
         "backend_node_initialized=false transport_ready=false";
}

bool IsaacMotionBackend::submit(
  const arm_cell_interfaces::action::ExecuteTask::Goal & goal)
{
  return transport_ && transport_->submit(goal);
}

uint8_t IsaacMotionBackend::execute_task(
  const arm_cell_interfaces::action::ExecuteTask::Goal & goal)
{
  if (!available()) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  if (!submit(goal)) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  if (goal.task_type.value != arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }
  switch (transport_->wait_for_motion_completion()) {
    case MotionCompletionOutcome::COMPLETED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
    case MotionCompletionOutcome::CANCELED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED;
    case MotionCompletionOutcome::SAFETY_PREEMPTED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    case MotionCompletionOutcome::TRANSPORT_UNAVAILABLE:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    case MotionCompletionOutcome::TIMEOUT:
    case MotionCompletionOutcome::STALE_FEEDBACK:
    default:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
}

uint8_t IsaacMotionBackend::execute_normalized(const NormalizedMotionRequest & request)
{
  if (!available()) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  if (!transport_->submit_normalized(request)) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  switch (transport_->wait_for_motion_completion()) {
    case MotionCompletionOutcome::COMPLETED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
    case MotionCompletionOutcome::CANCELED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED;
    case MotionCompletionOutcome::SAFETY_PREEMPTED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    case MotionCompletionOutcome::TRANSPORT_UNAVAILABLE:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    case MotionCompletionOutcome::TIMEOUT:
    case MotionCompletionOutcome::STALE_FEEDBACK:
    default:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
}

void IsaacMotionBackend::cancel()
{
  if (transport_) {
    if (transport_->gripper_active()) {
      transport_->stop_gripper();
    }
    transport_->cancel();
  }
}

void IsaacMotionBackend::safety_preempt(uint8_t stop_mode)
{
  if (gripper_port_ && gripper_port_->active()) {
    gripper_port_->stop();
  } else if (transport_ && transport_->gripper_active()) {
    transport_->stop_gripper();
  }
  if (transport_) {
    transport_->safety_preempt(stop_mode);
  }
}

void IsaacMotionBackend::hold()
{
  if (transport_) {
    if (transport_->gripper_active()) {
      transport_->stop_gripper();
    }
    transport_->hold();
  }
}

void IsaacMotionBackend::stop()
{
  if (transport_) {
    if (transport_->gripper_active()) {
      transport_->stop_gripper();
    }
    transport_->stop();
  }
}

bool IsaacMotionBackend::execution_active() const
{
  return transport_ && transport_->execution_active();
}

bool IsaacMotionBackend::backend_inactivity_confirmed() const
{
  return transport_ && transport_->inactivity_confirmed();
}

uint8_t IsaacMotionBackend::command_gripper(float width_mm)
{
  if (width_mm < 0.0F || width_mm > kFullyOpenWidthMm) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  }
  if (!available()) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  const auto normalized_opening = static_cast<double>(width_mm / kFullyOpenWidthMm);
  return transport_->command_normalized_gripper(normalized_opening) ?
         arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS :
         arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
}

void IsaacMotionBackend::begin_pick_grasp_relation()
{
  if (transport_) {
    transport_->begin_pick_grasp_relation();
  }
}

void IsaacMotionBackend::stage_selected_pick_grasp_relation(
  const geometry_msgs::msg::Transform & relation)
{
  if (transport_) {
    transport_->stage_selected_pick_grasp_relation(relation);
  }
}

void IsaacMotionBackend::discard_pending_grasp_relation()
{
  if (transport_) {
    transport_->discard_pending_grasp_relation();
  }
}

bool IsaacMotionBackend::confirm_fresh_held_grasp_relation()
{
  return transport_ && transport_->confirm_fresh_held_grasp_relation();
}

void IsaacMotionBackend::confirm_fresh_released_grasp_relation()
{
  if (transport_) {
    transport_->confirm_fresh_released_grasp_relation();
  }
}

bool IsaacMotionBackend::gripper_active() const
{
  return transport_ && transport_->gripper_active();
}

void IsaacMotionBackend::stop_gripper()
{
  if (transport_) {
    transport_->stop_gripper();
  }
}

}  // namespace arm_cell_motion_moveit2
