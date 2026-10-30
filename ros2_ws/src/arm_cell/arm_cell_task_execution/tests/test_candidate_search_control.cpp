#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <gtest/gtest.h>

#include <deque>
#include <string>
#include <vector>

#include "arm_cell_task_execution/candidate_search_control.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
namespace
{
class ScriptNode final : public BT::StatefulActionNode
{
public:
  using Callback = std::function<BT::NodeStatus()>;
  ScriptNode(const std::string & name, const BT::NodeConfiguration & config, Callback callback)
  : BT::StatefulActionNode(name, config), callback_(std::move(callback)) {}
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus onStart() override {return callback_();}
  BT::NodeStatus onRunning() override {return callback_();}
  void onHalted() override {}
  Callback callback_;
};

void register_script(
  BT::BehaviorTreeFactory & factory, const std::string & name,
  ScriptNode::Callback callback)
{
  factory.registerBuilder<ScriptNode>(
    name,
    [callback](const std::string & node_name, const BT::NodeConfiguration & config) {
      return std::make_unique<ScriptNode>(node_name, config, callback);
    });
}

BT::Tree make_tree(BT::BehaviorTreeFactory & factory)
{
  register_candidate_search_control(factory);
  return factory.createTreeFromText(
    "<root main_tree_to_execute=\"Search\"><BehaviorTree ID=\"Search\">"
    "<SearchCandidates><Select/><Attempt/><MayContinue/></SearchCandidates>"
    "</BehaviorTree></root>");
}
}  // namespace

TEST(CandidateSearchControl, RepeatsFailedAttemptsOnlyWhenTransitionIsAllowed)
{
  BT::BehaviorTreeFactory factory;
  int selections = 0;
  int attempts = 0;
  int transitions = 0;
  register_script(
    factory, "Select", [&]() {
      return ++selections <= 3 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_script(
    factory, "Attempt", [&]() {
      ++attempts;
      return attempts == 3 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_script(
    factory, "MayContinue", [&]() {
      ++transitions;
      return BT::NodeStatus::SUCCESS;
    });
  auto tree = make_tree(factory);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(selections, 3);
  EXPECT_EQ(attempts, 3);
  EXPECT_EQ(transitions, 2);
}

TEST(CandidateSearchControl, ExhaustionAndForbiddenTransitionRemainFailure)
{
  BT::BehaviorTreeFactory factory;
  int attempts = 0;
  register_script(factory, "Select", []() {return BT::NodeStatus::SUCCESS;});
  register_script(
    factory, "Attempt", [&]() {
      ++attempts;
      return BT::NodeStatus::FAILURE;
    });
  register_script(factory, "MayContinue", []() {return BT::NodeStatus::FAILURE;});
  auto tree = make_tree(factory);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(attempts, 1);
}

TEST(CandidateSearchControl, PropagatesRunningWithoutStartingAnotherCandidate)
{
  BT::BehaviorTreeFactory factory;
  int selections = 0;
  int attempts = 0;
  register_script(
    factory, "Select", [&]() {
      ++selections;
      return BT::NodeStatus::SUCCESS;
    });
  register_script(
    factory, "Attempt", [&]() {
      return ++attempts == 1 ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
    });
  register_script(factory, "MayContinue", []() {return BT::NodeStatus::FAILURE;});
  auto tree = make_tree(factory);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(selections, 1);
}

TEST(CandidateSearchControl, RetryControlKeepsTerminalAttemptStatus)
{
  BT::BehaviorTreeFactory factory;
  int attempts = 0;
  int transitions = 0;
  register_candidate_search_control(factory);
  register_script(
    factory, "Attempt", [&]() {
      return ++attempts < 3 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
  register_script(
    factory, "Transition", [&]() {
      ++transitions;
      return BT::NodeStatus::SUCCESS;
    });
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Retry\"><BehaviorTree ID=\"Retry\">"
    "<RetryWhileAllowed><Attempt/><Transition/></RetryWhileAllowed>"
    "</BehaviorTree></root>");

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(attempts, 3);
  EXPECT_EQ(transitions, 2);
}

TEST(CandidateSearchControl, RecoveryRunsAfterFailureWithoutReplacingPrimaryFailure)
{
  BT::BehaviorTreeFactory factory;
  int primary_calls = 0;
  int recovery_calls = 0;
  register_candidate_search_control(factory);
  register_script(
    factory, "Primary", [&]() {
      ++primary_calls;
      return BT::NodeStatus::FAILURE;
    });
  register_script(
    factory, "Recovery", [&]() {
      ++recovery_calls;
      return BT::NodeStatus::SUCCESS;
    });
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Recovery\"><BehaviorTree ID=\"Recovery\">"
    "<RecoverOnFailure><Primary/><Recovery/></RecoverOnFailure>"
    "</BehaviorTree></root>");

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(primary_calls, 1);
  EXPECT_EQ(recovery_calls, 1);
}

TEST(CandidateSearchControl, FinalizerRunsWithoutReplacingPrimaryStatus)
{
  BT::BehaviorTreeFactory factory;
  std::vector<std::string> events;
  register_candidate_search_control(factory);
  register_script(
    factory, "Primary", [&]() {
      events.push_back("primary");
      return BT::NodeStatus::FAILURE;
    });
  register_script(
    factory, "Finalize", [&]() {
      events.push_back("finalize");
      return BT::NodeStatus::SUCCESS;
    });
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Finalize\"><BehaviorTree ID=\"Finalize\">"
    "<RunAndFinalize><Primary/><Finalize/></RunAndFinalize>"
    "</BehaviorTree></root>");

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(events, (std::vector<std::string>{"primary", "finalize"}));
}
}  // namespace arm_cell_task_execution
