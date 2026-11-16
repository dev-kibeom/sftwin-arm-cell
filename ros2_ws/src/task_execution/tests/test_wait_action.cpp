#include <thread>
#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "arm_cell_task_execution/wait_action.hpp"
using namespace arm_cell_task_execution;
TEST(WaitAction, RemainsRunningWithoutBlockingAndHonorsSafetyBeforeCompletion)
{
  BT::BehaviorTreeFactory factory;
  std::uint8_t interruption = MotionTaskResultCode::TASK_RESULT_SUCCESS;
  TaskExecutionReport report;
  register_wait_action(factory, [&] {return interruption;}, report);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Test\"><BehaviorTree ID=\"Test\">"
    "<Wait duration_ms=\"2000\"/></BehaviorTree></root>");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
  interruption = MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(report.result_code, interruption);
}
TEST(WaitAction, HaltRestartsTimerAndZeroDurationChecksCancel)
{
  BT::BehaviorTreeFactory factory;
  std::uint8_t interruption = MotionTaskResultCode::TASK_RESULT_SUCCESS;
  TaskExecutionReport report;
  register_wait_action(factory, [&] {return interruption;}, report);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Test\"><BehaviorTree ID=\"Test\">"
    "<Wait duration_ms=\"0\"/></BehaviorTree></root>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  tree.haltTree();
  interruption = MotionTaskResultCode::TASK_RESULT_CANCELED;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(report.result_code, interruption);
}
TEST(WaitAction, HaltDiscardsElapsedTimeAndCompletionWaitsForDeadline)
{
  BT::BehaviorTreeFactory factory;
  TaskExecutionReport report;
  register_wait_action(factory, {}, report);
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Test\"><BehaviorTree ID=\"Test\">"
    "<Wait duration_ms=\"30\"/></BehaviorTree></root>");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  tree.haltTree();
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}
