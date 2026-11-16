#include <thread>
#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "arm_cell_task_execution/wait_action.hpp"
#include "arm_cell_task_execution/release_wait_action.hpp"
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

TEST(WaitAction, PredicateCompletionNeverSucceedsFromTimeoutAlone)
{
  TaskExecutionReport report;
  BT::BehaviorTreeFactory factory;
  bool ready = false;
  bool timed_out = false;
  factory.registerBuilder<WaitAction>(
    "Observe", [&](const auto & name, const auto & config) {
      return std::make_unique<WaitAction>(
        name, config, WaitAction::Checker{}, report,
        [&] {return ready;}, [&] {timed_out = true;});
    });
  auto tree = factory.createTreeFromText(
    R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
    <Observe duration_ms="0"/></BehaviorTree></root>)");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(timed_out);
  tree.haltTree();
  ready = true;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

namespace
{
class ObservedGripper final : public arm_cell_motion::GripperPort
{
public:
  std::uint8_t close() override {return MotionTaskResultCode::TASK_RESULT_SUCCESS;}
  std::uint8_t open() override {return MotionTaskResultCode::TASK_RESULT_SUCCESS;}
  arm_cell_motion::HoldingObservation holding() const override {return observation;}
  bool active() const override {return true;}
  void stop() override {}
  arm_cell_motion::HoldingObservation observation;
};
}

TEST(ReleaseWaitAction, RequiresNewAndFreshReleasedObservation)
{
  using namespace arm_cell_motion;
  auto gripper = std::make_shared<ObservedGripper>();
  TaskExecutionSupport support(nullptr, gripper, {});
  std::uint64_t baseline = 3;
  HoldingWaitResult result = HoldingWaitResult::TIMEOUT;
  TaskExecutionReport report;
  BT::BehaviorTreeFactory factory;
  register_release_wait_action(factory, "ObserveRelease", support, baseline, result, report);
  auto tree = factory.createTreeFromText(
    R"(<root main_tree_to_execute="Test"><BehaviorTree ID="Test">
    <ObserveRelease/></BehaviorTree></root>)");
  gripper->observation = {HoldingState::RELEASED, HoldingObservation::Clock::now(), 3, {}};
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  gripper->observation.sequence = 4;
  gripper->observation.observed_at -= std::chrono::seconds(1);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  gripper->observation.observed_at = HoldingObservation::Clock::now();
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(result, HoldingWaitResult::CONFIRMED);
}
