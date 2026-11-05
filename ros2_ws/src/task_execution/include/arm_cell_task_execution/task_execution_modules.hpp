#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <behaviortree_cpp_v3/bt_factory.h>

#include "arm_cell_task_execution/task_execution_support.hpp"
#include "arm_cell_task_execution/task_execution_report.hpp"
#include "arm_cell_motion/candidate_search.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

class PickCandidateSearch final
{
public:
  explicit PickCandidateSearch(
    TaskExecutionSupport & support,
    CandidatePriorityPolicy priority_policy = CandidatePriorityPolicy::DISTANCE_FIRST)
  : support_(support), priority_policy_(priority_policy) {}
  void register_nodes(
    BT::BehaviorTreeFactory & factory, BT::Blackboard::Ptr blackboard,
    TaskExecutionReport & report);
  const std::vector<std::string> & attempted_candidate_ids() const;
  const std::string & selected_candidate_id() const;
  void record_search();

private:
  struct State
  {
    std::vector<std::string> attempted_candidate_ids;
    std::string outcome_material_candidate_id;
    std::vector<arm_cell_interfaces::msg::PickCandidate> ordered_candidates;
    std::vector<MaterialCandidateSearchAttempt> attempts;
    ExecuteTask::Goal selected_goal;
    CandidateSearchCursor candidate_cursor;
    std::uint8_t last_result{MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED};
    bool direct_pick_pending{false};
    bool recording_enabled{false};
    bool candidate_active{false};
  };

  std::uint8_t prepare(
    const ExecuteTask::Goal & goal, TaskExecutionReport & report);
  bool select_next(
    const ExecuteTask::Goal & goal, BT::Blackboard & blackboard);
  void record_attempt(
    std::uint8_t result, const std::string & failure_stage, bool insertion_started);
  bool transition_allowed() const;
  TaskExecutionSupport & support_;
  CandidatePriorityPolicy priority_policy_;
  State state_;
  BT::Blackboard::Ptr blackboard_;
  TaskExecutionReport * report_{nullptr};
};

class PickExecution final
{
public:
  explicit PickExecution(TaskExecutionSupport & support)
  : support_(support) {}
  void register_nodes(
    BT::BehaviorTreeFactory & factory, BT::Blackboard::Ptr blackboard,
    TaskExecutionReport & report);

private:
  struct State
  {
    ExecuteTask::Goal goal;
    MotionPlan pending_motion_plan;
    std::uint64_t holding_baseline{0};
    bool candidate_inputs_validated{false};
    bool approach_submitted{false};
    bool released_confirmed{false};
    bool holding_confirmation_failed{false};
  };

  std::uint8_t prepare_candidate(bool candidate_inputs_validated);
  std::uint8_t open_and_confirm_release();
  std::uint8_t execute_approach(std::uint8_t motion_result);
  std::uint8_t execute_grasp_insertion(std::uint8_t motion_result);
  std::uint8_t close_and_confirm_grasp();
  std::uint8_t execute_retract(std::uint8_t motion_result);
  std::uint8_t record_stage(
    std::uint8_t result, const std::string & stage, bool held, bool retract_succeeded);
  std::uint8_t finalize_retract();

  TaskExecutionSupport & support_;
  State state_;
  BT::Blackboard::Ptr blackboard_;
  TaskExecutionReport * report_{nullptr};
};

class PickRecovery final
{
public:
  explicit PickRecovery(TaskExecutionSupport & support)
  : support_(support) {}
  void register_nodes(
    BT::BehaviorTreeFactory & factory, TaskExecutionReport & report);

private:
  struct State
  {
    bool already_returned{false};
    bool recovery_needed{false};
    bool open_required{false};
    std::uint64_t holding_baseline{0};
  };

  void prepare();
  std::uint8_t open_return();
  std::uint8_t confirm_return();

  TaskExecutionSupport & support_;
  State state_;
  TaskExecutionReport * report_{nullptr};
};

class PlaceExecution final
{
public:
  explicit PlaceExecution(TaskExecutionSupport & support)
  : support_(support) {}
  void register_nodes(
    BT::BehaviorTreeFactory & factory, BT::Blackboard::Ptr blackboard,
    TaskExecutionReport & report);

private:
  struct State
  {
    ExecuteTask::Goal goal;
    MotionPlan pending_motion_plan;
    std::uint8_t release_attempt{0};
    HoldingWaitResult holding_result{HoldingWaitResult::TIMEOUT};
    bool waypoint_strategy{false};
    bool waypoint_clearance_needed{false};
    geometry_msgs::msg::PoseStamped waypoint_clearance_target;
  };

  std::uint8_t validate_goal();
  std::uint8_t prepare_waypoint_clearance();
  std::uint8_t verify_waypoint_clearance();
  std::uint8_t execute_waypoint_home();
  std::uint8_t execute_approach(std::uint8_t motion_result);
  std::uint8_t execute_insertion(std::uint8_t motion_result);
  bool advance_planning_candidate();
  void begin_release_retries();
  std::uint8_t execute_release_attempt();
  bool release_confirmed() const;
  bool continue_release_retries() const;
  std::uint8_t finalize_release();
  std::uint8_t execute_retreat(std::uint8_t motion_result);

  TaskExecutionSupport & support_;
  State state_;
  BT::Blackboard::Ptr blackboard_;
  TaskExecutionReport * report_{nullptr};
};

class PlaceRecovery final
{
public:
  explicit PlaceRecovery(TaskExecutionSupport & support)
  : support_(support) {}
  void register_nodes(
    BT::BehaviorTreeFactory & factory, const ExecuteTask::Goal & goal,
    TaskExecutionReport & report);

private:
  struct State
  {
    ExecuteTask::Goal goal;
    MotionPortRequest pending_motion_request;
    MotionPlan pending_motion_plan;
    std::uint8_t original_retreat_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
    std::uint8_t original_result{MotionTaskResultCode::TASK_RESULT_INVALID_GOAL};
    std::uint64_t holding_baseline{0};
    bool retreat_result_captured{false};
    bool retreat_replanned{false};
    bool return_needed{false};
    bool approach_completed{false};
    bool approach_return_open_needed{false};
  };

  bool approach_recovery_eligible() const;
  bool observe_approach_recovery_holding();
  std::uint8_t open_approach_return();
  std::uint8_t confirm_approach_return();
  void prepare_held_failure_recovery();
  std::uint8_t return_held_home();
  std::uint8_t finalize_held_return();
  bool advance_retreat_recovery();
  std::uint8_t execute_retreat_retry(std::uint8_t motion_result);
  std::uint8_t finalize_retreat();

  TaskExecutionSupport & support_;
  State state_;
  TaskExecutionReport * report_{nullptr};
};

class HeldObjectReturn final
{
public:
  explicit HeldObjectReturn(TaskExecutionSupport & support)
  : support_(support) {}
  void register_nodes(
    BT::BehaviorTreeFactory & factory, const ExecuteTask::Goal & goal,
    TaskExecutionReport & report);

private:
  std::uint8_t execute();

  TaskExecutionSupport & support_;
  std::string stage_prefix_;
  TaskExecutionReport * report_{nullptr};
};

}  // namespace arm_cell_task_execution
