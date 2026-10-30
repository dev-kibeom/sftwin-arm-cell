#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "arm_cell_task_execution/motion_plan_execute_action.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
namespace
{
class RecordingMotionPort final : public MotionPort
{
public:
  MotionPlanningResult planMotion(const MotionPortRequest & request) override
  {
    calls.push_back("plan:" + request.movement_id);
    MotionPlanningResult result;
    result.result_code = plan_result;
    result.plan.plan_id = 1;
    result.plan.execution_id = request.execution_id;
    result.plan.movement_id = request.movement_id;
    return result;
  }
  MotionPortResult executePlannedMotion(const MotionPlan & plan) override
  {
    calls.push_back("execute:" + plan.movement_id);
    MotionPortResult result;
    result.result_code = execute_result;
    return result;
  }
  MotionPortResult planAndExecuteMotion(const MotionPortRequest &) override
  {
    ADD_FAILURE() << "legacy combined API must not be called";
    MotionPortResult result;
    return result;
  }
  MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID &, const std::string &) override
  {MotionCancelResult result; return result;}
  MotionObservation observeMotionState() const override {MotionObservation result; return result;}

  std::uint8_t plan_result{arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS};
  std::uint8_t execute_result{arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS};
  std::vector<std::string> calls;
};

struct MotionActionContext
{
  MotionPortRequest request;
  MotionPlan plan;
  std::uint8_t result_code{0};
};

BT::Tree make_plan_tree(BT::BehaviorTreeFactory & factory)
{
  return factory.createTreeFromText(
    "<root main_tree_to_execute=\"Plan\"><BehaviorTree ID=\"Plan\">"
    "<PlanMotion/></BehaviorTree></root>");
}

BT::Tree make_execute_tree(BT::BehaviorTreeFactory & factory)
{
  return factory.createTreeFromText(
    "<root main_tree_to_execute=\"Execute\"><BehaviorTree ID=\"Execute\">"
    "<ExecutePlannedMotion/></BehaviorTree></root>");
}

BT::BehaviorTreeFactory make_factory(
  TaskExecutionSupport & support, MotionActionContext & context)
{
  BT::BehaviorTreeFactory factory;
  context.request.execution_id.uuid[0] = 1;
  context.request.movement_id = "pick_approach";
  register_plan_motion_action(
    factory, "PlanMotion", support,
    [&context]() {return context.request;},
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_PLANNING,
    [&context](
      const MotionPlanningResult & result) {
      context.result_code = result.result_code;
      if (result.result_code != arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return BT::NodeStatus::FAILURE;
      }
      context.plan = result.plan;
      return BT::NodeStatus::SUCCESS;
    });
  register_execute_planned_motion_action(
    factory, "ExecutePlannedMotion", support,
    [&context]() {return context.plan;}, true,
    [&context](
      const MotionPortResult & result) {
      context.result_code = result.result_code;
      return result.result_code == arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  return factory;
}
}  // namespace

TEST(MotionPlanExecuteAction, PlansBeforeExecutingAndReturnsMeasuredExecutionResult)
{
  auto port = std::make_shared<RecordingMotionPort>();
  TaskExecutionSupport support(port, nullptr, MotionGeometry{});
  MotionActionContext context;
  auto factory = make_factory(support, context);
  auto plan_tree = make_plan_tree(factory);
  auto execute_tree = make_execute_tree(factory);

  EXPECT_EQ(plan_tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(port->calls, (std::vector<std::string>{"plan:pick_approach"}));
  EXPECT_EQ(context.plan.plan_id, 1U);
  EXPECT_EQ(execute_tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(port->calls, (std::vector<std::string>{"plan:pick_approach", "execute:pick_approach"}));
  EXPECT_EQ(
    context.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(MotionPlanExecuteAction, PlanningFailureNeverStartsTrajectoryExecution)
{
  auto port = std::make_shared<RecordingMotionPort>();
  port->plan_result = arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
  TaskExecutionSupport support(port, nullptr, MotionGeometry{});
  MotionActionContext context;
  auto factory = make_factory(support, context);
  auto plan_tree = make_plan_tree(factory);

  EXPECT_EQ(plan_tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(port->calls, (std::vector<std::string>{"plan:pick_approach"}));
  EXPECT_EQ(
    context.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
}
TEST(MotionPlanExecuteAction, PreservesPlanningAndSystemResultCodes)
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  for (const auto code : {Code::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED,
      Code::TASK_RESULT_IK_UNREACHABLE, Code::TASK_RESULT_PATH_OBSTRUCTED,
      Code::TASK_RESULT_BACKEND_ERROR, Code::TASK_RESULT_BACKEND_UNAVAILABLE,
      Code::TASK_RESULT_CANCELED, Code::TASK_RESULT_SAFETY_PREEMPTED})
  {
    SCOPED_TRACE(static_cast<int>(code));
    auto port = std::make_shared<RecordingMotionPort>();
    port->plan_result = code;
    TaskExecutionSupport support(port, nullptr, MotionGeometry{});
    MotionActionContext context;
    auto factory = make_factory(support, context);
    auto tree = make_plan_tree(factory);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(context.result_code, code);
    EXPECT_EQ(port->calls, (std::vector<std::string>{"plan:pick_approach"}));
  }
}
}  // namespace arm_cell_task_execution
