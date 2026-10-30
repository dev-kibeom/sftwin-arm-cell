#pragma once

#include <cstdint>
#include <memory>

#include <arm_cell_interfaces/msg/stop_mode.hpp>

#include "arm_cell_motion/motion_backend.hpp"

namespace arm_cell_motion
{

enum class MotionCompletionOutcome : uint8_t
{
  COMPLETED,
  CANCELED,
  SAFETY_PREEMPTED,
  JOINT_PROGRESS_STALLED,
  TIMEOUT,
  STALE_FEEDBACK,
  TRANSPORT_UNAVAILABLE
};

class IsaacMotionTransport
{
public:
  virtual ~IsaacMotionTransport() = default;

  virtual bool available() const = 0;
  virtual void begin_pick_search() {}
  virtual void begin_pick_search(const std::string & execution_id)
  { (void)execution_id; begin_pick_search(); }
  virtual void begin_pick_candidate_search() {}
  virtual void record_material_candidate_search(
    const std::vector<MaterialCandidateSearchAttempt> &, const std::string &) {}
  virtual bool apply_runtime_tuning(const MotionTuning &) {return true;}
  virtual bool set_motion_envelope(const MotionEnvelope & envelope)
  {
    return envelope.valid && envelope.max_velocity_scale == 1.0F &&
           envelope.max_acceleration_scale == 1.0F;
  }
  virtual MotionEnvelope applied_motion_envelope() const
  {
    return MotionEnvelope{1.0F, 1.0F, true};
  }
  virtual std::string availability_diagnostic() const {return "transport_readiness=unknown";}
  virtual bool submit(
    const arm_cell_interfaces::action::ExecuteTask::Goal & goal) = 0;
  virtual bool submit(
    const arm_cell_interfaces::action::ExecuteTask::Goal & goal,
    const std::string & motion_goal_uuid)
  {
    (void)motion_goal_uuid;
    return submit(goal);
  }
  virtual bool submit_normalized(const NormalizedMotionRequest & request) = 0;
  virtual uint8_t plan_normalized(
    const NormalizedMotionRequest &, NormalizedTrajectoryPlan &)
  {return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;}
  virtual uint8_t generate_place_waypoint_approach(
    const NormalizedMotionRequest &, const std::string &)
  {return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;}
  virtual bool execute_planned(const NormalizedTrajectoryPlan &)
  {return false;}
  virtual uint8_t execute_joint_target()
  {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  virtual MotionDisturbanceParameters disturbance_parameters() const {return {};}
  virtual void reset_pick_grasp_insertion_started() {}
  virtual bool pick_grasp_insertion_started() const {return false;}
  virtual void record_task_accuracy(const NormalizedMotionRequest &, uint8_t) {}
  virtual std::string last_pick_candidate_evaluation() const {return "";}
  virtual std::string last_place_failure_stage() const {return {};}
  virtual bool advance_to_next_planning_candidate() {return false;}
  virtual bool retry_place_retreat() {return false;}
  virtual void finalize_motion_planning_diagnostic(uint8_t) {}
  virtual void finalize_motion_planning_diagnostic(uint8_t code, double)
  {finalize_motion_planning_diagnostic(code);}
  virtual void finalize_motion_planning_diagnostic(
    uint8_t code, double elapsed_s, const std::string & failure_stage,
    const std::string & failure_detail)
  {
    (void)failure_stage;
    (void)failure_detail;
    finalize_motion_planning_diagnostic(code, elapsed_s);
  }
  virtual uint8_t normalized_failure_result() const
  {return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;}
  virtual MotionCompletionOutcome wait_for_motion_completion() = 0;
  uint8_t wait_for_joint_target_result()
  {
    using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
    switch (wait_for_motion_completion()) {
      case MotionCompletionOutcome::COMPLETED: return Code::TASK_RESULT_SUCCESS;
      case MotionCompletionOutcome::CANCELED: return Code::TASK_RESULT_CANCELED;
      case MotionCompletionOutcome::SAFETY_PREEMPTED: return Code::TASK_RESULT_SAFETY_PREEMPTED;
      case MotionCompletionOutcome::JOINT_PROGRESS_STALLED:
        return Code::TASK_RESULT_JOINT_PROGRESS_STALLED;
      case MotionCompletionOutcome::TRANSPORT_UNAVAILABLE:
        return Code::TASK_RESULT_BACKEND_UNAVAILABLE;
      default: return Code::TASK_RESULT_BACKEND_ERROR;
    }
  }

  virtual void cancel() = 0;
  virtual void safety_preempt(uint8_t stop_mode) = 0;
  virtual void hold() = 0;
  virtual void stop() = 0;
  virtual bool execution_active() const = 0;
  virtual bool inactivity_confirmed() const = 0;
  virtual std::string inactivity_diagnostic() const {return {};}
  virtual bool measured_joint_state(sensor_msgs::msg::JointState &) const {return false;}
  virtual bool measured_tcp_pose(geometry_msgs::msg::PoseStamped &) const {return false;}
  virtual bool command_normalized_gripper(double opening) = 0;
  virtual bool command_gripper_close() {return false;}
  virtual void begin_pick_grasp_relation() {}
  virtual void stage_selected_pick_grasp_relation(const geometry_msgs::msg::Transform &) {}
  virtual bool stage_pick_capture_reference(
    const geometry_msgs::msg::PoseStamped &, double, uint8_t, const std::string &) {return false;}
  virtual void discard_pending_grasp_relation() {}
  virtual bool commit_pending_grasp_relation() {return true;}
  virtual void clear_grasp_relation_after_release() {}
  virtual bool gripper_active() const = 0;
  virtual HoldingObservation holding_observation() const {return {};}
  virtual void stop_gripper() = 0;
};

class IsaacMotionBackend final : public MotionBackend
{
public:
  explicit IsaacMotionBackend(std::shared_ptr<IsaacMotionTransport> transport);

  bool available() const override;
  void begin_pick_search() override;
  void begin_pick_search(const std::string & execution_id) override;
  void begin_pick_candidate_search() override;
  void record_material_candidate_search(
    const std::vector<MaterialCandidateSearchAttempt> & attempts,
    const std::string & outcome) override;
  bool apply_runtime_tuning(const MotionTuning & tuning) override;
  bool set_motion_envelope(const MotionEnvelope & envelope) override;
  MotionEnvelope applied_motion_envelope() const override;
  bool measured_tcp_pose(geometry_msgs::msg::PoseStamped & pose) const override;
  std::shared_ptr<GripperPort> gripper_port() const override;
  std::string availability_diagnostic() const override;
  bool submit(const arm_cell_interfaces::action::ExecuteTask::Goal & goal) override;
  uint8_t execute_task(const arm_cell_interfaces::action::ExecuteTask::Goal & goal) override;
  uint8_t execute_task(
    const arm_cell_interfaces::action::ExecuteTask::Goal & goal,
    const std::string & motion_goal_uuid) override;
  uint8_t execute_normalized(const NormalizedMotionRequest & request) override;
  bool supports_split_planning() const override {return true;}
  uint8_t plan_normalized(
    const NormalizedMotionRequest & request, NormalizedTrajectoryPlan & plan) override;
  uint8_t generate_place_waypoint_approach(
    const NormalizedMotionRequest & request, const std::string & path) override;
  uint8_t execute_planned_normalized(const NormalizedTrajectoryPlan & plan) override;
  uint8_t execute_joint_target() override;
  MotionDisturbanceParameters disturbance_parameters() const override;
  void reset_pick_grasp_insertion_started() override;
  bool pick_grasp_insertion_started() const override;
  std::string last_pick_candidate_evaluation() const override;
  std::string last_place_failure_stage() const override;
  bool advance_to_next_planning_candidate() override;
  bool retry_place_retreat() override;
  void finalize_motion_planning_diagnostic(uint8_t result_code) override;
  void finalize_motion_planning_diagnostic(uint8_t result_code, double elapsed_s) override;
  void finalize_motion_planning_diagnostic(
    uint8_t result_code, double elapsed_s, const std::string & failure_stage,
    const std::string & failure_detail) override;
  uint8_t normalized_failure_result() const override;
  void cancel() override;
  void safety_preempt(uint8_t stop_mode) override;
  void hold() override;
  void stop() override;
  bool execution_active() const override;
  bool backend_inactivity_confirmed() const override;
  std::string inactivity_diagnostic() const override;
  bool measured_joint_state(sensor_msgs::msg::JointState & state) const override;
  uint8_t command_gripper(float width_mm) override;
  void begin_pick_grasp_relation() override;
  void stage_selected_pick_grasp_relation(const geometry_msgs::msg::Transform & relation) override;
  bool stage_pick_capture_reference(
    const geometry_msgs::msg::PoseStamped & pose, double width_mm, uint8_t symmetry_order,
    const std::string & execution_id) override;
  void discard_pending_grasp_relation() override;
  bool commit_pending_grasp_relation() override;
  void clear_grasp_relation_after_release() override;
  bool gripper_active() const override;
  void stop_gripper() override;

private:
  std::shared_ptr<IsaacMotionTransport> transport_;
  std::shared_ptr<GripperPort> gripper_port_;
};

}  // namespace arm_cell_motion
