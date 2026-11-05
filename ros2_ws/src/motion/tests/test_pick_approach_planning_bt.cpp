#include <gtest/gtest.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <chrono>
#include <memory>

#include "arm_cell_motion/pick_approach_planning_bt.hpp"

namespace arm_cell_motion
{
namespace
{
// Test callbacks bind directly into factory builders; production has concrete node types.
struct TestActions
{
  using Action = std::function<BT::NodeStatus(PlanningContext &)>;
  std::map<std::string, Action> actions;
};
class TestAction final : public PlanningLeaf
{
public:
  TestAction(
    const std::string & name, const BT::NodeConfiguration & config,
    std::string stage, TestActions::Action action)
  : PlanningLeaf(name, config), stage_(std::move(stage)), action_(std::move(action)) {}

private:
  const char * stage_name() const override {return stage_.c_str();}
  BT::NodeStatus execute(PlanningContext & context) override {return action_(context);}
  std::string stage_;
  TestActions::Action action_;
};
void bind_test_actions(BT::BehaviorTreeFactory & factory, const TestActions & actions)
{
  for (const auto & entry : actions.actions) {
    factory.unregisterBuilder(entry.first);
    factory.registerBuilder<TestAction>(
      entry.first,
      [stage = entry.first, action = entry.second](const std::string & name,
      const BT::NodeConfiguration & config) {
        return std::make_unique<TestAction>(name, config, stage, action);
      });
  }
}
BT::Tree make_search_tree(
  BT::BehaviorTreeFactory & factory, const std::shared_ptr<PlanningContext> & context,
  const std::shared_ptr<TestActions> & operations)
{
  register_pick_approach_planning_nodes(factory);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", context);
  bind_test_actions(factory, *operations);
  return factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Search\">"
    "<BehaviorTree ID=\"Search\"><SearchPoseSeed><Attempt/>"
    "</SearchPoseSeed></BehaviorTree></root>", blackboard);
}

std::shared_ptr<PlanningContext> make_context(
  std::size_t poses, std::size_t seeds, double budget_s = 5.0)
{
  auto context = std::make_shared<PlanningContext>();
  context->planning_budget = std::make_unique<PickSearchBudget>(budget_s);
  context->search_controller = std::make_unique<PickSearchController>(poses, seeds);
  context->interruption_result = []() {return std::uint8_t{0};};
  return context;
}
}  // namespace

TEST(PickApproachPlanningBT, FactoryBuildsConcreteProductionActionTypes)
{
  BT::BehaviorTreeFactory factory;
  register_pick_approach_planning_nodes(factory);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", make_context(1U, 2U));
  auto tree = factory.createTreeFromText(pick_approach_planning_tree_xml(), blackboard);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<PrepareAttempt *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<PrepareSeed *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<SetGraspScenePolicy *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<SolveGraspIK *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<ValidateGraspKinematics *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<ValidateGraspContact *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<ValidateGraspCollision *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<SetApproachScenePolicy *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<SolveApproachIK *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<ValidateApproachState *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<RegisterValidCandidate *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<SetPathScenePolicy *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<PlanApproachPath *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<FinalizeTrajectory *>(node.get()) != nullptr;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      tree.nodes.begin(), tree.nodes.end(), [](const auto & node) {
        return dynamic_cast<RecordAttemptOutcome *>(node.get()) != nullptr;
      }), 1);
}

TEST(PickApproachPlanningBT, RetryReplacesAttemptAndPreservesRequestState)
{
  auto context = make_context(1U, 2U);
  auto * budget = context->planning_budget.get();
  auto operations = std::make_shared<TestActions>();
  std::size_t calls = 0U;
  operations->actions["Attempt"] = [&](PlanningContext & current) {
      EXPECT_TRUE(current.attempt.grasp_joint_positions.empty());
      EXPECT_TRUE(current.attempt.approach_joint_positions.empty());
      EXPECT_TRUE(current.attempt.failure_reason.empty());
      EXPECT_TRUE(current.attempt.planning_request_evidence.empty());
      EXPECT_EQ(current.attempt.planning_status, "NOT_RUN");
      EXPECT_EQ(current.planning_budget.get(), budget);
      ++calls;
      if (calls == 1U) {
        current.attempt.grasp_joint_positions = {1.0};
        current.attempt.approach_joint_positions = {2.0};
        current.attempt.failure_reason = "collision";
        current.attempt.planning_status = "FAILED";
        current.attempt.planning_request_evidence.push_back("previous request");
        current.attempt.attempt_evaluation = PickSearchEvaluation::GRASP_COLLISION;
        return BT::NodeStatus::FAILURE;
      }
      current.result.selected = true;
      return BT::NodeStatus::SUCCESS;
    };
  BT::BehaviorTreeFactory factory;
  auto tree = make_search_tree(factory, context, operations);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls, 2U);
  EXPECT_FALSE(context->diagnostic_evidence.empty());
}

TEST(PickApproachPlanningBT, RepeatsSamePoseAfterCollisionAndAcceptsFirstExecutablePlan)
{
  auto context = make_context(1U, 2U);
  auto operations = std::make_shared<TestActions>();
  std::size_t attempts = 0U;
  operations->actions["Attempt"] = [&](PlanningContext & current) {
      ++attempts;
      if (attempts == 1U) {
        current.attempt.attempt_evaluation = PickSearchEvaluation::GRASP_COLLISION;
        current.attempt.failure_stage = "GRASP_COLLISION";
        current.attempt.failure_reason = "contact pair";
        return BT::NodeStatus::FAILURE;
      }
      current.result.selected = true;
      return BT::NodeStatus::SUCCESS;
    };
  BT::BehaviorTreeFactory factory;
  auto tree = make_search_tree(factory, context, operations);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(attempts, 2U);
  EXPECT_EQ(context->search_controller->attempt_count(), 2U);
  EXPECT_EQ(context->search_controller->status(), PickSearchStatus::SELECTED);
}

TEST(PickApproachPlanningBT, DefersNoSolutionPoseAndRevisitsItAfterMeasuredSeeds)
{
  auto context = make_context(2U, 2U);
  auto operations = std::make_shared<TestActions>();
  std::vector<std::pair<std::size_t, std::size_t>> visits;
  operations->actions["Attempt"] = [&](PlanningContext & current) {
      visits.emplace_back(
        current.search_controller->pose_index(), current.search_controller->seed_attempt());
      if (visits.size() == 2U) {
        current.result.selected = true;
        return BT::NodeStatus::SUCCESS;
      }
      current.attempt.attempt_evaluation = PickSearchEvaluation::GRASP_IK_NO_SOLUTION;
      return BT::NodeStatus::FAILURE;
    };
  BT::BehaviorTreeFactory factory;
  auto tree = make_search_tree(factory, context, operations);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(visits, (std::vector<std::pair<std::size_t, std::size_t>>{{0U, 1U}, {1U, 1U}}));
}

TEST(PickApproachPlanningBT, RevisitsDeferredPoseAfterMeasuredSeedsOnOtherPoses)
{
  auto context = make_context(2U, 2U);
  auto operations = std::make_shared<TestActions>();
  std::vector<std::pair<std::size_t, std::size_t>> visits;
  operations->actions["Attempt"] = [&](PlanningContext & current) {
      visits.emplace_back(
        current.search_controller->pose_index(), current.search_controller->seed_attempt());
      if (visits.size() == 3U) {
        current.result.selected = true;
        return BT::NodeStatus::SUCCESS;
      }
      current.attempt.attempt_evaluation = PickSearchEvaluation::GRASP_IK_NO_SOLUTION;
      return BT::NodeStatus::FAILURE;
    };
  BT::BehaviorTreeFactory factory;
  auto tree = make_search_tree(factory, context, operations);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    visits, (std::vector<std::pair<std::size_t, std::size_t>>{
      {0U, 1U}, {1U, 1U}, {0U, 2U}}));
}

TEST(PickApproachPlanningBT, StopsAtBudgetAndInterruptionBeforeStartingAnotherAttempt)
{
  auto context = make_context(4U, 2U, 0.0);
  auto operations = std::make_shared<TestActions>();
  std::size_t attempts = 0U;
  operations->actions["Attempt"] = [&](PlanningContext &) {
      ++attempts;
      return BT::NodeStatus::FAILURE;
    };
  BT::BehaviorTreeFactory factory;
  auto tree = make_search_tree(factory, context, operations);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(attempts, 0U);
  EXPECT_EQ(context->attempt.failure_reason, "SEARCH_BUDGET_EXHAUSTED");

  context = make_context(4U, 2U);
  context->interruption_result = []() {return std::uint8_t{1};};
  BT::BehaviorTreeFactory interrupted_factory;
  auto interrupted_tree = make_search_tree(interrupted_factory, context, operations);
  EXPECT_EQ(interrupted_tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(attempts, 0U);
  EXPECT_EQ(context->attempt.failure_stage, "INTERRUPTED");
}

TEST(PickApproachPlanningBT, ProductionSequenceStopsAfterFailedStageAndRecordsOutcome)
{
  for (const auto evaluation : {PickSearchEvaluation::GRASP_CONTACT_MISMATCH,
      PickSearchEvaluation::GLOBAL_PLANNING_FAILED, PickSearchEvaluation::BACKEND_ERROR})
  {
    auto context = make_context(2U, 2U);
    auto operations = std::make_shared<TestActions>();
    std::size_t solves = 0U;
    std::size_t records = 0U;
    operations->actions["PrepareAttempt"] = [](PlanningContext &) {
        return BT::NodeStatus::SUCCESS;
      };
    operations->actions["PrepareSeed"] = [](PlanningContext &) {return BT::NodeStatus::SUCCESS;};
    operations->actions["SetGraspScenePolicy"] = [](PlanningContext &) {
        return BT::NodeStatus::SUCCESS;
      };
    operations->actions["SolveGraspIK"] = [&](PlanningContext & current) {
        ++solves;
        current.attempt.attempt_evaluation = evaluation;
        current.attempt.failure_stage = "GRASP_IK";
        current.attempt.failure_reason = "fixture failure";
        return BT::NodeStatus::FAILURE;
      };
    context->record_attempt_outcome = [&](auto, auto, auto recorded, const auto &, const auto &) {
        ++records;
        EXPECT_EQ(recorded, evaluation);
      };
    BT::BehaviorTreeFactory factory;
    register_pick_approach_planning_nodes(factory);
    auto blackboard = BT::Blackboard::create();
    blackboard->set("context", context);
    bind_test_actions(factory, *operations);
    auto tree = factory.createTreeFromText(pick_approach_planning_tree_xml(), blackboard);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(solves, evaluation == PickSearchEvaluation::BACKEND_ERROR ? 1U : 2U);
    EXPECT_EQ(records, solves);
    EXPECT_EQ(context->attempt.executed_stages.back(), "SolveGraspIK");
  }
}

TEST(PickApproachPlanningBT, CancelAfterAttemptDiscardsSelectedPlanAndRecordsInterruption)
{
  auto context = make_context(2U, 2U);
  auto operations = std::make_shared<TestActions>();
  bool cancelled = false;
  context->interruption_result = [&]() {return std::uint8_t(cancelled ? 1 : 0);};
  operations->actions["Attempt"] = [&](PlanningContext & current) {
      current.result.selected = true;
      current.result.selected_joint_goal = {1.0};
      cancelled = true;
      return BT::NodeStatus::SUCCESS;
    };
  std::size_t records = 0;
  context->record_attempt_outcome = [&](auto, auto, auto, const auto & stage, const auto &) {
      ++records;
      EXPECT_EQ(stage, "INTERRUPTED");
    };
  BT::BehaviorTreeFactory factory;
  auto tree = make_search_tree(factory, context, operations);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(context->result.selected);
  EXPECT_TRUE(context->result.selected_joint_goal.empty());
  EXPECT_EQ(context->result.search_status, PickSearchStatus::INTERRUPTED);
  EXPECT_EQ(records, 1U);
}

TEST(PickApproachPlanningBT, BudgetAtCompletionDiscardsPlanButPreservesBackendFailure)
{
  for (const bool backend_error : {false, true}) {
    auto context = make_context(2U, 2U);
    auto operations = std::make_shared<TestActions>();
    operations->actions["Attempt"] = [&](PlanningContext & current) {
        current.planning_budget = std::make_unique<PickSearchBudget>(0.0);
        current.result.selected = !backend_error;
        current.attempt.attempt_evaluation = backend_error ? PickSearchEvaluation::BACKEND_ERROR :
          PickSearchEvaluation::EXECUTABLE_APPROACH_SELECTED;
        return backend_error ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
      };
    BT::BehaviorTreeFactory factory;
    auto tree = make_search_tree(factory, context, operations);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(context->result.selected);
    EXPECT_EQ(
      context->result.search_status, backend_error ? PickSearchStatus::BACKEND_ERROR :
      PickSearchStatus::SEARCH_BUDGET_EXHAUSTED);
    EXPECT_EQ(context->search_controller->attempt_count(), 1U);
  }
}

TEST(PickApproachPlanningBT, ProductionTreeExposesEachAttemptStageAsAnOrderedLeaf)
{
  auto context = make_context(1U, 2U);
  auto operations = std::make_shared<TestActions>();
  std::vector<std::string> steps;
  std::size_t attempts = 0U;
  const std::vector<std::string> attempt_steps{
    "PrepareAttempt", "PrepareSeed", "SetGraspScenePolicy", "SolveGraspIK",
    "ValidateGraspKinematics",
    "ValidateGraspContact", "ValidateGraspCollision", "SetApproachScenePolicy",
    "SolveApproachIK", "ValidateApproachState", "RegisterValidCandidate", "SetPathScenePolicy",
    "PlanApproachPath",
    "FinalizeTrajectory", "RecordAttemptOutcome"};
  for (const auto & step : attempt_steps) {
    operations->actions[step] = [&, step](PlanningContext & current) {
        steps.push_back(step);
        if (step == "RecordAttemptOutcome" && ++attempts == 1U) {
          current.attempt.attempt_evaluation = PickSearchEvaluation::GRASP_COLLISION;
          return BT::NodeStatus::FAILURE;
        }
        if (step == "RecordAttemptOutcome") {current.result.selected = true;}
        return BT::NodeStatus::SUCCESS;
      };
  }
  BT::BehaviorTreeFactory factory;
  register_pick_approach_planning_nodes(factory);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", context);
  bind_test_actions(factory, *operations);
  auto tree = factory.createTreeFromText(pick_approach_planning_tree_xml(), blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(attempts, 2U);
  std::vector<std::string> expected;
  for (int attempt = 0; attempt < 2; ++attempt) {
    expected.insert(expected.end(), attempt_steps.begin(), attempt_steps.end());
  }
  EXPECT_EQ(steps, expected);
}
}  // namespace arm_cell_motion
