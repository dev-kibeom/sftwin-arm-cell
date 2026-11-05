#include <gtest/gtest.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <memory>
#include <string>
#include <vector>

#include "arm_cell_task_execution/task_execution_support.hpp"
#include "arm_cell_task_execution/task_execution_modules.hpp"
#include "legacy_backend_motion_port.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
namespace
{
class ModuleBackend final : public MotionBackend
{
public:
  bool available() const override {return true;}
  bool submit(const ExecuteTask::Goal &) override {return true;}
  uint8_t execute_normalized(const NormalizedMotionRequest & request) override
  {
    ++normalized_requests;
    phases.push_back(request.phase);
    events.push_back("execute_normalized");
    return normalized_result;
  }
  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return false;}
  bool backend_inactivity_confirmed() const override {return true;}

  void begin_pick_candidate_search() override {++candidate_starts;}
  void record_material_candidate_search(
    const std::vector<MaterialCandidateSearchAttempt> & attempts,
    const std::string & outcome) override
  {recorded_attempts = attempts; recorded_outcome = outcome;}
  void clear_grasp_relation_after_release() override {++released_relations;}
  bool retry_place_retreat() override
  {++retreat_retry_requests; events.push_back("retry_place_retreat"); return allow_retreat_retry;}

  int candidate_starts{0};
  int normalized_requests{0};
  int released_relations{0};
  int retreat_retry_requests{0};
  bool allow_retreat_retry{false};
  std::uint8_t normalized_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  std::vector<MaterialCandidateSearchAttempt> recorded_attempts;
  std::vector<std::uint8_t> phases;
  std::vector<std::string> events;
  std::string recorded_outcome;
};

class ModuleGripper final : public GripperPort
{
public:
  uint8_t close() override
  {state = HoldingState::HELD; ++sequence; return MotionTaskResultCode::TASK_RESULT_SUCCESS;}
  uint8_t open() override
  {
    state = HoldingState::RELEASED; ++sequence; ++open_count;
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }
  HoldingObservation holding() const override
  {return HoldingObservation{state, HoldingObservation::Clock::now(), sequence, std::nullopt};}
  bool active() const override {return true;}
  void stop() override {}

  HoldingState state{HoldingState::UNKNOWN};
  std::uint64_t sequence{0};
  std::size_t open_count{0};
};

struct Fixture
{
  Fixture()
  : backend(std::make_shared<ModuleBackend>()), gripper(std::make_shared<ModuleGripper>()),
    support(
      std::make_shared<test::LegacyBackendMotionPort>(backend, MotionGeometry{}),
      gripper, MotionGeometry{}, std::chrono::milliseconds(1)),
    blackboard(BT::Blackboard::create())
  {
    report.motion_execution_id = "module-test";
    blackboard->set("goal", ExecuteTask::Goal{});
  }

  std::shared_ptr<ModuleBackend> backend;
  std::shared_ptr<ModuleGripper> gripper;
  TaskExecutionSupport support;
  TaskExecutionReport report;
  BT::Blackboard::Ptr blackboard;
};

arm_cell_interfaces::msg::PickCandidate candidate(
  const std::string & id, double height)
{
  arm_cell_interfaces::msg::PickCandidate result;
  result.acquisition_id = "acquisition";
  result.candidate_id = id;
  result.top_height_tolerance_m = 0.01;
  result.target_pose.header.frame_id = "base_link";
  result.target_pose.pose.position.z = height;
  result.target_pose.pose.orientation.w = 1.0;
  result.orientation_symmetry_order = ExecuteTask::Goal::ORIENTATION_UNIQUE;
  return result;
}

}  // namespace

TEST(PickCandidateSearchModule, OwnsOrderingAndPublishesSelectedCandidate)
{
  Fixture fixture;
  ExecuteTask::Goal goal;
  goal.material_candidates = {candidate("lower", 0.1), candidate("higher", 0.2)};
  fixture.blackboard->set("goal", goal);

  PickCandidateSearch module(fixture.support);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, fixture.blackboard, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Search\"><BehaviorTree ID=\"Search\">"
    "<Sequence><PreparePickSearch/><SelectNextPickCandidate/></Sequence>"
    "</BehaviorTree></root>", fixture.blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    fixture.blackboard->get<ExecuteTask::Goal>("pick_candidate_goal").vision_candidate_id,
    "higher");
  EXPECT_EQ(module.attempted_candidate_ids(), (std::vector<std::string>{"higher"}));
  EXPECT_EQ(fixture.backend->candidate_starts, 1);
}

TEST(PickCandidateSearchModule, UnifiedHighMediumCandidatesUseDistanceBeforeHeight)
{
  Fixture fixture;
  // A/C were classified HIGH and B MEDIUM by Vision. Confidence stays diagnostic.
  auto a = candidate("A", 0.3);
  auto b = candidate("B", 0.1);
  auto c = candidate("C", 0.4);
  a.target_pose.pose.position.y = 0.2;
  b.target_pose.pose.position.y = 0.3;
  c.target_pose.pose.position.y = 0.1;
  ExecuteTask::Goal goal;
  goal.material_candidates = {a, b, c};
  fixture.blackboard->set("goal", goal);
  PickCandidateSearch module(fixture.support);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, fixture.blackboard, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Search\"><BehaviorTree ID=\"Search\">"
    "<Sequence><PreparePickSearch/><SelectNextPickCandidate/>"
    "<SelectNextPickCandidate/><SelectNextPickCandidate/></Sequence>"
    "</BehaviorTree></root>", fixture.blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(module.attempted_candidate_ids(), (std::vector<std::string>{"B", "A", "C"}));
  EXPECT_EQ(
    fixture.blackboard->get<ExecuteTask::Goal>("pick_candidate_goal").vision_candidate_id, "C");
}

TEST(PickCandidateSearchModule, AcceptsHeightFirstPolicyAndKeepsVisionTieOrder)
{
  Fixture fixture;
  auto first = candidate("vision-first", 0.3);
  auto second = candidate("vision-second", 0.3);
  first.target_pose.pose.position.y = -10.0;
  second.target_pose.pose.position.y = 10.0;
  auto shorter = candidate("shorter", 0.2);
  shorter.target_pose.pose.position.y = 100.0;
  ExecuteTask::Goal goal;
  goal.material_candidates = {first, second, shorter};
  fixture.blackboard->set("goal", goal);

  PickCandidateSearch module(fixture.support, CandidatePriorityPolicy::HEIGHT_FIRST);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, fixture.blackboard, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Search\"><BehaviorTree ID=\"Search\">"
    "<Sequence><PreparePickSearch/><SelectNextPickCandidate/></Sequence>"
    "</BehaviorTree></root>", fixture.blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    fixture.blackboard->get<ExecuteTask::Goal>("pick_candidate_goal").vision_candidate_id,
    "vision-first");
}

TEST(PickExecutionModule, ValidationFailureIsAReportedDomainResult)
{
  Fixture fixture;
  fixture.blackboard->set("pick_candidate_goal", ExecuteTask::Goal{});
  fixture.blackboard->set("pick_candidate_inputs_validated", false);
  PickExecution module(fixture.support);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, fixture.blackboard, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Pick\"><BehaviorTree ID=\"Pick\">"
    "<PreparePickCandidate/></BehaviorTree></root>", fixture.blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(fixture.report.result_code, MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_EQ(fixture.report.failure_stage, "validation:TASK_RESULT_INVALID_GOAL");
}

TEST(PickRecoveryModule, OwnsFreshHoldingDecisionAndReturnOutcome)
{
  Fixture fixture;
  fixture.report.failure_stage = "grasp_motion";
  fixture.report.result_code = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
  fixture.gripper->state = HoldingState::HELD;
  fixture.gripper->sequence = 1;
  PickRecovery module(fixture.support);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Recovery\"><BehaviorTree ID=\"Recovery\">"
    "<Sequence><PreparePickRetractRecovery/><PickRetractRecoveryNeeded/>"
    "<PickRetractOpenRequired/></Sequence></BehaviorTree></root>", fixture.blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fixture.report.result_code, MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
}

TEST(PickRecoveryModule, ReturnsHeldObjectWithFreshReleaseAndPreservesPickFailure)
{
  Fixture fixture;
  fixture.report.failure_stage = "grasp_motion";
  fixture.report.result_code = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
  fixture.gripper->state = HoldingState::HELD;
  fixture.gripper->sequence = 1;
  PickRecovery module(fixture.support);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Recovery\"><BehaviorTree ID=\"Recovery\">"
    "<Sequence><PreparePickRetractRecovery/><PickRetractRecoveryNeeded/>"
    "<PickRetractOpenRequired/><OpenPickRetractReturn/><ConfirmPickRetractReturn/>"
    "</Sequence></BehaviorTree></root>");

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fixture.gripper->open_count, 1U);
  EXPECT_EQ(fixture.backend->released_relations, 1);
  EXPECT_EQ(fixture.report.result_code, MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(fixture.report.object_disposition, ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
}

TEST(PlaceExecutionModule, InvalidGoalStopsBeforeBackendExecution)
{
  Fixture fixture;
  PlaceExecution module(fixture.support);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, fixture.blackboard, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Place\"><BehaviorTree ID=\"Place\">"
    "<ValidatePlaceGoal/></BehaviorTree></root>", fixture.blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(fixture.report.result_code, MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_EQ(fixture.backend->normalized_requests, 0);
}

TEST(PlaceRecoveryModule, ApproachReleaseIsConfirmedWithoutRetreatCoupling)
{
  Fixture fixture;
  fixture.report.failure_stage = "place_approach_planning";
  fixture.report.result_code = MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
  fixture.gripper->state = HoldingState::RELEASED;
  fixture.gripper->sequence = 1;
  PlaceRecovery module(fixture.support);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, ExecuteTask::Goal{}, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Recovery\"><BehaviorTree ID=\"Recovery\">"
    "<Sequence><PlaceApproachRecoveryEligible/><ObservePlaceApproachRecoveryHolding/>"
    "<PlaceApproachRecoveryComplete/></Sequence></BehaviorTree></root>", fixture.blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fixture.report.result_code, MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(fixture.report.object_disposition, ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
  EXPECT_EQ(fixture.backend->released_relations, 1);
}

TEST(PlaceRecoveryModule, ClearanceFailureUsesExistingApproachRecovery)
{
  Fixture fixture;
  fixture.report.failure_stage = "place_clearance_planning";
  fixture.report.result_code = MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
  fixture.gripper->state = HoldingState::RELEASED;
  fixture.gripper->sequence = 1;
  PlaceRecovery module(fixture.support);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, ExecuteTask::Goal{}, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Recovery\"><BehaviorTree ID=\"Recovery\">"
    "<PlaceApproachRecoveryEligible/></BehaviorTree></root>", fixture.blackboard);

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

TEST(PlaceRecoveryModule, OwnsRetreatRetryAndPreservesOriginalFailure)
{
  Fixture fixture;
  fixture.report.failure_stage = "place_retract_backend_request";
  fixture.report.result_code = MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
  fixture.report.object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_PLACED;
  fixture.backend->normalized_result = MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
  fixture.backend->allow_retreat_retry = true;
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  PlaceRecovery module(fixture.support);
  BT::BehaviorTreeFactory factory;
  module.register_nodes(factory, goal, fixture.report);
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"3\" main_tree_to_execute=\"Recovery\"><BehaviorTree ID=\"Recovery\">"
    "<Sequence><AdvancePlaceRetreatRecovery/><PreparePlaceRetreatRetry/>"
    "<PlanPlaceRetreatRetry/><ExecutePlaceRetreatRetry/>"
    "<FinalizePlaceRetreat/></Sequence></BehaviorTree></root>");

  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(fixture.backend->retreat_retry_requests, 1);
  EXPECT_EQ(fixture.backend->normalized_requests, 1);
  EXPECT_EQ(
    fixture.backend->phases, (std::vector<std::uint8_t>{
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING}));
  EXPECT_EQ(
    fixture.backend->events, (std::vector<std::string>{
      "retry_place_retreat", "execute_normalized"}));
  EXPECT_EQ(fixture.report.failure_stage, "place_retract_replanning");
  EXPECT_EQ(fixture.report.result_code, MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(fixture.report.object_disposition, ExecuteTask::Result::OBJECT_DISPOSITION_PLACED);
}

}  // namespace arm_cell_task_execution
