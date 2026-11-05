#pragma once

#include <memory>
#include <unordered_map>

#include "arm_cell_motion/motion_execution_support.hpp"
#include "arm_cell_task_execution/task_execution_support.hpp"

namespace arm_cell_task_execution::test
{
class LegacyBackendMotionPort final : public arm_cell_motion::MotionPort
{
public:
  LegacyBackendMotionPort(
    std::shared_ptr<arm_cell_motion::MotionBackend> backend,
    arm_cell_motion::MotionGeometry geometry)
  : backend_(std::move(backend)), support_(backend_, std::move(geometry)) {}

  void set_interruption_checker(
    arm_cell_motion::MotionExecutionSupport::InterruptionChecker checker)
  {
    interruption_checker_ = std::move(checker); support_.set_interruption_checker(
      interruption_checker_);
  }

  arm_cell_motion::MotionPortResult planAndExecuteMotion(
    const arm_cell_motion::MotionPortRequest & request) override
  {
    const auto planning = planMotion(request);
    if (planning.result_code ==
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS)
    {
      return executePlannedMotion(planning.plan);
    }
    arm_cell_motion::MotionPortResult result;
    result.execution_id = request.execution_id;
    result.movement_id = request.movement_id;
    result.result_code = planning.result_code;
    return result;
  }
  arm_cell_motion::MotionPlanningResult planMotion(
    const arm_cell_motion::MotionPortRequest & request) override
  {
    arm_cell_motion::MotionPlanningResult result;
    result.plan.plan_id = next_plan_id_++;
    result.plan.execution_id = request.execution_id;
    result.plan.movement_id = request.movement_id;
    planned_requests_[result.plan.plan_id] = support_.normalized_request(request);
    result.result_code = arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
    return result;
  }
  arm_cell_motion::MotionPortResult executePlannedMotion(
    const arm_cell_motion::MotionPlan & plan) override
  {
    arm_cell_motion::MotionPortResult result;
    result.execution_id = plan.execution_id;
    result.movement_id = plan.movement_id;
    auto found = planned_requests_.find(plan.plan_id);
    if (found == planned_requests_.end()) {
      result.result_code = arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
      return result;
    }
    auto normalized = found->second;
    normalized.interruption_result = interruption_checker_;
    result.result_code = backend_->execute_normalized(normalized);
    planned_requests_.erase(found);
    if (interruption_checker_) {
      const auto interrupted = interruption_checker_();
      if (interrupted != arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        result.result_code = interrupted;
      }
    }
    result.motion_execution_completed =
      result.result_code == arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
    return result;
  }
  arm_cell_motion::MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID &, const std::string &) override
  {
    backend_->cancel(); return {true, backend_->backend_inactivity_confirmed(),
      arm_cell_interfaces::msg::MotionStatus()};
  }
  bool measuredTcpPose(geometry_msgs::msg::PoseStamped & pose) const override
  {return backend_->measured_tcp_pose(pose);}
  arm_cell_motion::MotionObservation observeMotionState() const override
  {
    return {arm_cell_interfaces::msg::MotionStatus(), false, sensor_msgs::msg::JointState()};
  }
  void beginPickSearch(const std::string & id) override {backend_->begin_pick_search(id);}
  void beginPickCandidateSearch() override {backend_->begin_pick_candidate_search();}
  void recordMaterialCandidateSearch(
    const std::vector<arm_cell_motion::MaterialCandidateSearchAttempt> & attempts,
    const std::string & outcome) override
  {backend_->record_material_candidate_search(attempts, outcome);}
  std::string lastPickCandidateEvaluation() const override
  {return backend_->last_pick_candidate_evaluation();}
  void resetPickGraspInsertionStarted() override {backend_->reset_pick_grasp_insertion_started();}
  bool pickGraspInsertionStarted() const override {return backend_->pick_grasp_insertion_started();}
  std::string lastPlaceFailureStage() const override {return backend_->last_place_failure_stage();}
  bool advanceToNextPlanningCandidate() override
  {
    return backend_->advance_to_next_planning_candidate();
  }
  bool retryPlaceRetreat() override {return backend_->retry_place_retreat();}
  void finalizeMotionPlanningDiagnostic(
    std::uint8_t code, double elapsed, const std::string & stage, const std::string & detail)
  override {backend_->finalize_motion_planning_diagnostic(code, elapsed, stage, detail);}
  void beginPickGraspRelation() override {backend_->begin_pick_grasp_relation();}
  void stageSelectedPickGraspRelation(const geometry_msgs::msg::Transform & t) override
  {backend_->stage_selected_pick_grasp_relation(t);}
  bool stagePickCaptureReference(
    const geometry_msgs::msg::PoseStamped & p, double h, std::uint8_t s,
    const std::string & source) override
  {return backend_->stage_pick_capture_reference(p, h, s, source);}
  void discardPendingGraspRelation() override {backend_->discard_pending_grasp_relation();}
  bool commitPendingGraspRelation() override {return backend_->commit_pending_grasp_relation();}
  void clearGraspRelationAfterRelease() override {backend_->clear_grasp_relation_after_release();}
  std::uint8_t executeLegacyMovement(
    std::uint8_t task_type, const std::string & id,
    const std::string & movement) override
  {
    (void)movement;
    arm_cell_interfaces::action::ExecuteTask::Goal goal;
    goal.task_type.value = task_type;
    return support_.execute_task(goal, id);
  }
  std::uint8_t executePlaceRecoveryJointTarget(
    const unique_identifier_msgs::msg::UUID &, const std::string &) override
  {return support_.execute_joint_target();}

private:
  std::shared_ptr<arm_cell_motion::MotionBackend> backend_;
  arm_cell_motion::MotionExecutionSupport support_;
  arm_cell_motion::MotionExecutionSupport::InterruptionChecker interruption_checker_;
  std::unordered_map<std::uint64_t, arm_cell_motion::NormalizedMotionRequest> planned_requests_;
  std::uint64_t next_plan_id_{1};
};
}  // namespace arm_cell_task_execution::test
