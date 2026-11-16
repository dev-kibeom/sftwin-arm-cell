#include <gtest/gtest.h>
#include "arm_cell_task_execution/taught_motion_action.hpp"

using namespace arm_cell_task_execution;

namespace
{
class TaughtPort final : public MotionPort
{
public:
  std::vector<std::string> motions;
  std::vector<std::string> movement_ids;
  std::uint8_t result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  std::uint8_t executeTaughtMotion(
    const std::string & motion, const std::string & execution_id,
    const std::string & movement_id) override
  {
    EXPECT_EQ(execution_id, "00000000000000000000000000000001");
    motions.push_back(motion);
    movement_ids.push_back(movement_id);
    return result;
  }
  MotionPortResult planAndExecuteMotion(const MotionPortRequest &) override
  {
    return MotionPortResult();
  }
  MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID &, const std::string &) override
  {
    return MotionCancelResult();
  }
  MotionObservation observeMotionState() const override {return MotionObservation();}
};
}

TEST(TaughtMotionAction, ComposesNamedActionsInSequence)
{
  auto port = std::make_shared<TaughtPort>();
  TaskExecutionSupport support(port, nullptr, {});
  support.set_execution_identity("00000000000000000000000000000001");
  TaskExecutionReport report;
  BT::BehaviorTreeFactory factory;
  register_taught_motion_action(factory, support, report);
  auto tree = factory.createTreeFromText(
    R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main"><Sequence>
      <MoveTaught motion="home"/><MoveTaught motion="standby"/>
    </Sequence></BehaviorTree></root>)");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(port->motions, (std::vector<std::string>{"home", "standby"}));
  ASSERT_EQ(port->movement_ids.size(), 2U);
  EXPECT_NE(port->movement_ids[0], port->movement_ids[1]);
}

TEST(TaughtMotionAction, MaterialReturnUsesTheSameNamedMovementPort)
{
  auto port = std::make_shared<TaughtPort>();
  TaskExecutionSupport support(port, nullptr, {});
  support.set_execution_identity("00000000000000000000000000000001");
  TaskExecutionReport report;
  BT::BehaviorTreeFactory factory;
  register_taught_motion_action(factory, support, report);
  auto tree = factory.createTreeFromText(
    R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main"><Sequence>
      <MoveTaught motion="return_material"/><MoveTaught motion="arbitrary_pose"/>
    </Sequence></BehaviorTree></root>)");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(port->motions, (std::vector<std::string>{"return_material", "arbitrary_pose"}));
}

TEST(TaughtMotionAction, FailureStopsTheFollowingAction)
{
  auto port = std::make_shared<TaughtPort>();
  port->result = MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
  TaskExecutionSupport support(port, nullptr, {});
  support.set_execution_identity("00000000000000000000000000000001");
  TaskExecutionReport report;
  BT::BehaviorTreeFactory factory;
  register_taught_motion_action(factory, support, report);
  auto tree = factory.createTreeFromText(
    R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main"><Sequence>
      <MoveTaught motion="home"/><MoveTaught motion="standby"/>
    </Sequence></BehaviorTree></root>)");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(port->motions, (std::vector<std::string>{"home"}));
  EXPECT_EQ(report.result_code, MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
}

TEST(TaughtMotionAction, LatchedInterruptionPreventsSubmission)
{
  auto port = std::make_shared<TaughtPort>();
  TaskExecutionSupport support(port, nullptr, {});
  support.set_interruption_checker([] {return MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;});
  TaskExecutionReport report;
  BT::BehaviorTreeFactory factory;
  register_taught_motion_action(factory, support, report);
  auto tree = factory.createTreeFromText(
    R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
      <MoveTaught motion="home"/>
    </BehaviorTree></root>)");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(port->motions.empty());
  EXPECT_EQ(report.result_code, MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
}
