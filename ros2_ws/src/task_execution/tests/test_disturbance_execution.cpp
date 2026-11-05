#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "arm_cell_task_execution/motion_bt_executor.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
namespace
{
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;

class DisturbanceMotionPort final : public MotionPort
{
public:
  MotionPlanningResult planMotion(const MotionPortRequest & request) override
  {
    MotionPlanningResult result;
    const auto index = planning_requests.size();
    planning_requests.push_back(request);
    events.push_back('P');
    result.plan.execution_id = request.execution_id;
    result.plan.movement_id = request.movement_id;
    result.plan.plan_id = index + 1;
    result.plan.end_state.name = {"joint"};
    result.plan.end_state.position = {static_cast<double>(index + 1)};
    result.result_code = index == failing_plan_index ? failure_code : Code::TASK_RESULT_SUCCESS;
    return result;
  }

  MotionPortResult executePlannedMotion(const MotionPlan & plan) override
  {
    MotionPortResult result;
    events.push_back('E');
    result.execution_id = plan.execution_id;
    result.movement_id = plan.movement_id;
    result.result_code = executed_plans.size() == failing_execute_index ?
      Code::TASK_RESULT_BACKEND_ERROR : Code::TASK_RESULT_SUCCESS;
    result.motion_execution_completed = result.result_code == Code::TASK_RESULT_SUCCESS;
    executed_plans.push_back(plan);
    return result;
  }

  MotionPortResult planAndExecuteMotion(const MotionPortRequest &) override
  {MotionPortResult result; return result;}

  MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID &, const std::string &) override
  {MotionCancelResult result; return result;}

  MotionObservation observeMotionState() const override
  {MotionObservation result; return result;}

  MotionDisturbanceParameters disturbanceParameters() const override
  {
    MotionDisturbanceParameters result;
    result.push_distance_m = 0.02;
    result.approach_clearance_m = 0.01;
    result.speed_fraction = 0.2;
    return result;
  }

  std::size_t failing_plan_index{static_cast<std::size_t>(-1)};
  std::size_t failing_execute_index{static_cast<std::size_t>(-1)};
  std::uint8_t failure_code{Code::TASK_RESULT_PATH_OBSTRUCTED};
  std::vector<MotionPortRequest> planning_requests;
  std::vector<MotionPlan> executed_plans;
  std::vector<char> events;
};

MotionDisturbRequest make_request()
{
  MotionDisturbRequest request;
  request.execution_id.uuid[0] = 1;
  request.movement_id = "disturb";
  request.acquisition_id = "acquisition";
  arm_cell_interfaces::msg::DisturbanceCandidate candidate;
  candidate.candidate_id = "candidate";
  candidate.center.header.frame_id = "base_link";
  candidate.center.pose.position.x = 0.4;
  candidate.center.pose.position.y = 0.2;
  candidate.center.pose.position.z = 0.5;
  candidate.center.pose.orientation.w = 1.0;
  candidate.observed_height_m = 0.3;
  request.disturbance_candidates.push_back(candidate);
  return request;
}
}  // namespace

TEST(DisturbanceExecution, PlansEverySegmentBeforeExecutingAndRetriesPlanningFailure)
{
  auto motion = std::make_shared<DisturbanceMotionPort>();
  motion->failing_plan_index = 0;
  TaskExecutionSupport support(motion, nullptr, MotionGeometry{});
  MotionBtExecutor executor(support);

  const auto result = executor.execute_disturb(make_request());

  ASSERT_EQ(
    result.outcome.value,
    arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_COMPLETED);
  ASSERT_EQ(motion->planning_requests.size(), 4U);
  EXPECT_NEAR(motion->planning_requests[0].target_pose.pose.position.x, 0.39, 1e-12);
  EXPECT_NEAR(motion->planning_requests[1].target_pose.pose.position.x, 0.41, 1e-12);
  EXPECT_NEAR(motion->planning_requests[2].target_pose.pose.position.x, 0.38, 1e-12);
  EXPECT_NEAR(motion->planning_requests[3].target_pose.pose.position.x, 0.37, 1e-12);
  EXPECT_EQ(motion->planning_requests[1].has_start_joint_state, false);
  EXPECT_EQ(motion->planning_requests[2].has_start_joint_state, true);
  EXPECT_EQ(motion->planning_requests[2].start_joint_state.position[0], 2.0);
  EXPECT_EQ(motion->planning_requests[3].start_joint_state.position[0], 3.0);
  EXPECT_EQ(motion->events, (std::vector<char>{'P', 'P', 'P', 'P', 'E', 'E', 'E'}));
}

TEST(DisturbanceExecution, DoesNotSearchAnotherDirectionAfterExecutionStarts)
{
  auto motion = std::make_shared<DisturbanceMotionPort>();
  motion->failing_execute_index = 0;
  TaskExecutionSupport support(motion, nullptr, MotionGeometry{});
  MotionBtExecutor executor(support);

  const auto result = executor.execute_disturb(make_request());

  EXPECT_EQ(
    result.outcome.value,
    arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED);
  EXPECT_EQ(motion->planning_requests.size(), 3U);
  EXPECT_EQ(motion->executed_plans.size(), 1U);
  EXPECT_EQ(motion->events, (std::vector<char>{'P', 'P', 'P', 'E'}));
}
}  // namespace arm_cell_task_execution
