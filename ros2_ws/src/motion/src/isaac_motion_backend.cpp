#include "arm_cell_motion/isaac_motion_backend.hpp"

#include "arm_cell_motion/isaac_gripper_port.hpp"

#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

namespace arm_cell_motion
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

void IsaacMotionBackend::begin_pick_search()
{
  if (transport_) {transport_->begin_pick_search();}
}

void IsaacMotionBackend::begin_pick_search(const std::string & execution_id)
{
  if (transport_) {transport_->begin_pick_search(execution_id);}
}

void IsaacMotionBackend::begin_pick_candidate_search()
{
  if (transport_) {transport_->begin_pick_candidate_search();}
}

void IsaacMotionBackend::record_material_candidate_search(
  const std::vector<MaterialCandidateSearchAttempt> & attempts,
  const std::string & outcome)
{
  if (transport_) {transport_->record_material_candidate_search(attempts, outcome);}
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

bool IsaacMotionBackend::measured_tcp_pose(geometry_msgs::msg::PoseStamped & pose) const
{
  return transport_ && transport_->measured_tcp_pose(pose);
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
  return execute_task(goal, {});
}

uint8_t IsaacMotionBackend::execute_task(
  const arm_cell_interfaces::action::ExecuteTask::Goal & goal,
  const std::string & motion_goal_uuid)
{
  if (!available()) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  if (!transport_->submit(goal, motion_goal_uuid)) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  if (goal.task_type.value != arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }
  return transport_->wait_for_joint_target_result();
}

uint8_t IsaacMotionBackend::execute_normalized(const NormalizedMotionRequest & request)
{
  if (!available()) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  if (!transport_->submit_normalized(request)) {
    return transport_->normalized_failure_result();
  }
  const auto outcome = transport_->wait_for_motion_completion();
  uint8_t execution_result = transport_->normalized_failure_result();
  switch (outcome) {
    case MotionCompletionOutcome::COMPLETED:
      execution_result = arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
      break;
    case MotionCompletionOutcome::CANCELED:
      execution_result = arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED;
      break;
    case MotionCompletionOutcome::SAFETY_PREEMPTED:
      execution_result =
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
      break;
    case MotionCompletionOutcome::TRANSPORT_UNAVAILABLE:
      execution_result =
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
      break;
    case MotionCompletionOutcome::JOINT_PROGRESS_STALLED:
      execution_result =
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED;
      break;
    case MotionCompletionOutcome::TIMEOUT:
    case MotionCompletionOutcome::STALE_FEEDBACK:
      break;
  }
  if ((request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ||
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE) &&
    (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING ||
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING ||
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING))
  {
    transport_->record_task_accuracy(request, execution_result);
  }
  switch (outcome) {
    case MotionCompletionOutcome::COMPLETED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
    case MotionCompletionOutcome::CANCELED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED;
    case MotionCompletionOutcome::SAFETY_PREEMPTED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    case MotionCompletionOutcome::TRANSPORT_UNAVAILABLE:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    case MotionCompletionOutcome::JOINT_PROGRESS_STALLED:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED;
    case MotionCompletionOutcome::TIMEOUT:
    case MotionCompletionOutcome::STALE_FEEDBACK:
    default:
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
}

uint8_t IsaacMotionBackend::plan_normalized(
  const NormalizedMotionRequest & request, NormalizedTrajectoryPlan & plan)
{
  if (!available()) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  return transport_->plan_normalized(request, plan);
}

uint8_t IsaacMotionBackend::generate_place_waypoint_approach(
  const NormalizedMotionRequest & request, const std::string & path)
{
  if (!available()) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  return transport_->generate_place_waypoint_approach(request, path);
}

uint8_t IsaacMotionBackend::execute_planned_normalized(
  const NormalizedTrajectoryPlan & plan)
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  if (!available()) {return Code::TASK_RESULT_BACKEND_UNAVAILABLE;}
  if (!transport_->execute_planned(plan)) {return transport_->normalized_failure_result();}
  const auto outcome = transport_->wait_for_motion_completion();
  uint8_t result = Code::TASK_RESULT_BACKEND_ERROR;
  switch (outcome) {
    case MotionCompletionOutcome::COMPLETED:
      result = Code::TASK_RESULT_SUCCESS;
      break;
    case MotionCompletionOutcome::CANCELED:
      result = Code::TASK_RESULT_CANCELED;
      break;
    case MotionCompletionOutcome::SAFETY_PREEMPTED:
      result = Code::TASK_RESULT_SAFETY_PREEMPTED;
      break;
    case MotionCompletionOutcome::TRANSPORT_UNAVAILABLE:
      result = Code::TASK_RESULT_BACKEND_UNAVAILABLE;
      break;
    case MotionCompletionOutcome::JOINT_PROGRESS_STALLED:
      result = Code::TASK_RESULT_JOINT_PROGRESS_STALLED;
      break;
    case MotionCompletionOutcome::TIMEOUT:
    case MotionCompletionOutcome::STALE_FEEDBACK:
      break;
  }
  const auto & request = plan.request;
  if ((request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ||
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE) &&
    (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING ||
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING ||
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING))
  {
    transport_->record_task_accuracy(request, result);
  }
  return result;
}

uint8_t IsaacMotionBackend::execute_joint_target()
{
  if (!available()) {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  return transport_->execute_joint_target();
}

MotionDisturbanceParameters IsaacMotionBackend::disturbance_parameters() const
{
  return transport_->disturbance_parameters();
}

void IsaacMotionBackend::reset_pick_grasp_insertion_started()
{
  if (transport_) {transport_->reset_pick_grasp_insertion_started();}
}

bool IsaacMotionBackend::pick_grasp_insertion_started() const
{
  return transport_ && transport_->pick_grasp_insertion_started();
}

std::string IsaacMotionBackend::last_pick_candidate_evaluation() const
{
  return transport_ ? transport_->last_pick_candidate_evaluation() : "";
}

std::string IsaacMotionBackend::last_place_failure_stage() const
{
  return transport_ ? transport_->last_place_failure_stage() : std::string{};
}

bool IsaacMotionBackend::advance_to_next_planning_candidate()
{
  return transport_ && transport_->advance_to_next_planning_candidate();
}

bool IsaacMotionBackend::retry_place_retreat()
{
  return transport_ && transport_->retry_place_retreat();
}

void IsaacMotionBackend::finalize_motion_planning_diagnostic(uint8_t result_code)
{
  if (transport_) {
    transport_->finalize_motion_planning_diagnostic(result_code);
  }
}

uint8_t IsaacMotionBackend::normalized_failure_result() const
{
  return transport_ ? transport_->normalized_failure_result() :
         arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
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

std::string IsaacMotionBackend::inactivity_diagnostic() const
{
  return transport_ ? transport_->inactivity_diagnostic() : "transport_unavailable";
}

bool IsaacMotionBackend::measured_joint_state(sensor_msgs::msg::JointState & state) const
{
  return transport_ && transport_->measured_joint_state(state);
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

bool IsaacMotionBackend::stage_pick_capture_reference(
  const geometry_msgs::msg::PoseStamped & pose, double width_mm, uint8_t symmetry_order,
  const std::string & execution_id)
{
  if (transport_) {
    return transport_->stage_pick_capture_reference(pose, width_mm, symmetry_order, execution_id);
  }
  return false;
}

void IsaacMotionBackend::discard_pending_grasp_relation()
{
  if (transport_) {
    transport_->discard_pending_grasp_relation();
  }
}

bool IsaacMotionBackend::commit_pending_grasp_relation()
{
  return transport_ && transport_->commit_pending_grasp_relation();
}

void IsaacMotionBackend::clear_grasp_relation_after_release()
{
  if (transport_) {
    transport_->clear_grasp_relation_after_release();
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

}  // namespace arm_cell_motion

namespace arm_cell_motion
{
void IsaacMotionBackend::finalize_motion_planning_diagnostic(uint8_t code, double elapsed_s)
{
  if (transport_) {transport_->finalize_motion_planning_diagnostic(code, elapsed_s);}
}

void IsaacMotionBackend::finalize_motion_planning_diagnostic(
  uint8_t code, double elapsed_s, const std::string & failure_stage,
  const std::string & failure_detail)
{
  if (transport_) {
    transport_->finalize_motion_planning_diagnostic(
      code, elapsed_s, failure_stage, failure_detail);
  }
}
}  // namespace arm_cell_motion
