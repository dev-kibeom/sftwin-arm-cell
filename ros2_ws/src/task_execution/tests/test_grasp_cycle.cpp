#include "arm_cell_task_execution/task_executor.hpp"
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
namespace
{
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
constexpr const char * execution_id = "1234567890abcdef1234567890abcdef";
class FixtureMotion : public MotionPort
{
public:
  std::vector<std::string> events;
  std::string fail_motion;
  std::uint8_t failure{Code::TASK_RESULT_PATH_OBSTRUCTED};
  int committed{0};
  int cleared{0};
  bool fail_clear{false};
  MotionPortResult planAndExecuteMotion(const MotionPortRequest &) override
  {
    return MotionPortResult();
  }
  MotionCancelResult cancelMotion(const unique_identifier_msgs::msg::UUID &,
                                  const std::string &) override
  {
    return MotionCancelResult();
  }
  MotionObservation observeMotionState() const override { return MotionObservation(); }
  std::uint8_t executeTaughtMotion(const std::string & name, const std::string &,
                                   const std::string &) override
  {
    events.push_back(name);
    return name == fail_motion ? failure : Code::TASK_RESULT_SUCCESS;
  }
  bool commitPendingGraspRelation() override
  {
    ++committed;
    return true;
  }
  void clearGraspRelationAfterRelease() override {if (fail_clear) {throw std::runtime_error("scene sync failed");} ++cleared; }
};
class FixtureTool : public GripperPort
{
public:
  FixtureMotion & motion;
  GraspStrategy strategy;
  HoldingState state{HoldingState::UNKNOWN};
  std::uint64_t sequence{0};
  bool confirm_grasp{true};
  GraspExecutionIdentity configured_identity;
  explicit FixtureTool(FixtureMotion & m, GraspStrategy s) : motion(m), strategy(s) {}
  bool supportsGraspStrategy(GraspStrategy requested) const override
  {
    return requested == strategy;
  }
  std::uint8_t prepareGrasp(const GraspSpecification & spec) override
  {
    return supportsGraspStrategy(spec.strategy) ? Code::TASK_RESULT_SUCCESS
                                                : Code::TASK_RESULT_INVALID_GOAL;
  }
  std::uint8_t prepareConfiguredGrasp(
    const GraspSpecification & spec, const GraspExecutionIdentity & identity) override
  {
    configured_identity = identity;
    return prepareGrasp(spec);
  }
  std::uint8_t grasp() override
  {
    motion.events.push_back(strategy == GraspStrategy::INTERNAL_RADIAL ? "expand" : "close");
    state = confirm_grasp ? HoldingState::HELD : HoldingState::UNKNOWN;
    ++sequence;
    return Code::TASK_RESULT_SUCCESS;
  }
  std::uint8_t release() override
  {
    motion.events.push_back(strategy == GraspStrategy::INTERNAL_RADIAL ? "contract" : "open");
    state = HoldingState::RELEASED;
    ++sequence;
    return Code::TASK_RESULT_SUCCESS;
  }
  std::uint8_t close() override
  {
    ADD_FAILURE() << "Physical close bypassed semantic intent";
    return 0;
  }
  std::uint8_t open() override
  {
    ADD_FAILURE() << "Physical open bypassed semantic intent";
    return 0;
  }
  HoldingObservation holding() const override
  {
    return {state, HoldingObservation::Clock::now(), sequence, std::nullopt};
  }
  bool active() const override { return false; }
  void stop() override {}
};
PreparedGraspCycle cycle(GraspStrategy strategy)
{
  PreparedGraspCycle request;
  request.grasp.strategy = strategy;
  request.grasp.insertion_depth_m = .004;
  request.grasp.bore_radius_m = .025;
  request.grasp.thickness_m = .002;
  request.grasp.release_position_m = .02;
  request.grasp.grasp_position_m = .01;
  request.approach_motion = "fixture_approach";
  request.insertion_motion = "fixture_insert";
  request.transfer_motion = "fixture_transfer";
  request.retreat_motion = "fixture_retreat";
  return request;
}
} // namespace
TEST(GraspCycle, SameTaskTreeRunsBoxAndBothRadialFingerConfigurations)
{
  for (const auto strategy : {GraspStrategy::BOX_EXTERIOR, GraspStrategy::INTERNAL_RADIAL})
  {
    for (const double depth : {.004, .020})
    {
      auto motion = std::make_shared<FixtureMotion>();
      auto tool = std::make_shared<FixtureTool>(*motion, strategy);
      TaskExecutor executor(motion, tool, {}, {}, std::chrono::milliseconds(1));
      auto request = cycle(strategy);
      request.grasp.insertion_depth_m = depth;
      EXPECT_EQ(executor.execute_grasp_cycle(request, execution_id), Code::TASK_RESULT_SUCCESS);
      EXPECT_EQ(
          motion->events,
          (std::vector<std::string>{
              strategy == GraspStrategy::INTERNAL_RADIAL ? "contract" : "open", "fixture_approach",
              "fixture_insert", strategy == GraspStrategy::INTERNAL_RADIAL ? "expand" : "close",
              "fixture_transfer", strategy == GraspStrategy::INTERNAL_RADIAL ? "contract" : "open",
              "fixture_retreat"}));
      EXPECT_EQ(motion->committed, 1);
      EXPECT_EQ(motion->cleared, 1);
      EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
    }
  }
}
TEST(GraspCycle, CapabilityMismatchAndInvalidBoreFailBeforeActuation)
{
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::BOX_EXTERIOR);
  TaskExecutor executor(motion, tool);
  EXPECT_EQ(executor.execute_grasp_cycle(cycle(GraspStrategy::INTERNAL_RADIAL), execution_id),
            Code::TASK_RESULT_INVALID_GOAL);
  auto request = cycle(GraspStrategy::BOX_EXTERIOR);
  request.insertion_motion.clear();
  EXPECT_EQ(executor.execute_grasp_cycle(request, execution_id), Code::TASK_RESULT_INVALID_GOAL);
  EXPECT_TRUE(motion->events.empty());
}
TEST(GraspCycle, UnconfirmedExpansionUsesExistingReleaseRecoveryWithoutTransfer)
{
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  tool->confirm_grasp = false;
  TaskExecutor executor(motion, tool, {}, {}, std::chrono::milliseconds(1));
  auto request = cycle(GraspStrategy::INTERNAL_RADIAL);
  request.identity.execution_id = execution_id;
  request.identity.supply_id = "supply";
  request.identity.material_id = "material";
  request.identity.slot_id = "slot_0";
  request.identity.supply_epoch = "epoch";
  request.identity.reservation_id = "reservation";
  EXPECT_EQ(executor.execute_grasp_cycle(request, execution_id),
            Code::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(motion->events, (std::vector<std::string>{"contract", "fixture_approach",
                                                      "fixture_insert", "expand", "contract"}));
  EXPECT_EQ(motion->committed, 0);
  EXPECT_FALSE(executor.released_at_configured_destination());
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
}
TEST(GraspCycle, SafetyAtInsertionDoesNotExpandOrReleaseAgain)
{
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  motion->fail_motion = "fixture_insert";
  motion->failure = Code::TASK_RESULT_SAFETY_PREEMPTED;
  TaskExecutor executor(motion, tool);
  EXPECT_EQ(executor.execute_grasp_cycle(cycle(GraspStrategy::INTERNAL_RADIAL), execution_id),
            Code::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(motion->events,
            (std::vector<std::string>{"contract", "fixture_approach", "fixture_insert"}));
}

TEST(GraspCycle, InvalidIdentityAndBoreAreRejectedBeforePreparation)
{
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  TaskExecutor executor(motion, tool);
  EXPECT_EQ(executor.execute_grasp_cycle(cycle(GraspStrategy::INTERNAL_RADIAL), ""),
            Code::TASK_RESULT_INVALID_GOAL);
  auto request = cycle(GraspStrategy::INTERNAL_RADIAL);
  request.grasp.insertion_depth_m = 0.;
  EXPECT_EQ(executor.execute_grasp_cycle(request, execution_id), Code::TASK_RESULT_INVALID_GOAL);
  EXPECT_TRUE(motion->events.empty());
}
TEST(GraspCycle, InsertionFailureRemainsRecordedAfterPreparedContextEnds)
{
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  motion->fail_motion = "fixture_insert";
  TaskExecutor executor(motion, tool);
  EXPECT_EQ(executor.execute_grasp_cycle(cycle(GraspStrategy::INTERNAL_RADIAL), execution_id),
            Code::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_TRUE(executor.pick_grasp_insertion_started());
}
TEST(GraspCycle, ConfirmedReleaseSurvivesRetreatFailure)
{
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  motion->fail_motion = "fixture_retreat";
  TaskExecutor executor(motion, tool);
  EXPECT_EQ(executor.execute_grasp_cycle(cycle(GraspStrategy::INTERNAL_RADIAL), execution_id),
            Code::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
  EXPECT_EQ(motion->cleared, 1);
}

TEST(GraspCycle, PreparedFixtureCannotBypassConfiguredMaterialForceAdmission)
{
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  TaskExecutor executor(motion, tool);
  executor.set_grip_force_policy(GripForcePolicy{});
  EXPECT_EQ(executor.execute_grasp_cycle(cycle(GraspStrategy::INTERNAL_RADIAL), execution_id),
            Code::TASK_RESULT_INVALID_GOAL);
  EXPECT_TRUE(motion->events.empty());
}
TEST(GraspCycle, RejectedNextOperationDoesNotExposePreviousInsertionEvidence)
{
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  TaskExecutor executor(motion, tool);
  auto request = cycle(GraspStrategy::INTERNAL_RADIAL);
  ASSERT_EQ(executor.execute_grasp_cycle(request, execution_id), Code::TASK_RESULT_SUCCESS);
  ASSERT_TRUE(executor.pick_grasp_insertion_started());
  request.insertion_motion.clear();
  EXPECT_EQ(executor.execute_grasp_cycle(request, execution_id), Code::TASK_RESULT_INVALID_GOAL);
  EXPECT_FALSE(executor.pick_grasp_insertion_started());
}
} // namespace arm_cell_task_execution

#include "arm_cell_task_execution/configured_grasp_catalog.hpp"
TEST(ConfiguredGraspCatalog, RejectsUntrustedCallerSlotAndMalformedTrustedGeometry)
{
  using namespace arm_cell_task_execution;
  EXPECT_THROW(parse_configured_grasp_cycles(R"({"slot":{}})"), std::invalid_argument);
  EXPECT_EQ(parse_configured_grasp_cycles("{}").find("unknown"), nullptr);
}

namespace arm_cell_task_execution
{
TEST(ConfiguredGraspExecution, SuppliesTrustedMetadataAndPreservesReleaseOnRetreatFailure)
{
  using namespace arm_cell_task_execution;
  using namespace arm_cell_motion;
  const auto catalog = parse_configured_grasp_cycles(R"({"slot_0":{
    "supply_id":"tape", "material_id":"Tape17",
    "grasp":{"strategy":"internal_radial","insertion_depth_m":0.012,"thickness_m":0.017,
      "inner_radius_m":0.025,"release_position_m":0.02,"grasp_position_m":0.01},
    "approach_motion":"fixture_approach","insertion_motion":"fixture_insert",
    "transfer_motion":"fixture_transfer","retreat_motion":"fixture_retreat"}})");
  ASSERT_EQ(catalog.find("other_slot"), nullptr);
  const auto * teaching = catalog.find("slot_0");
  ASSERT_NE(teaching, nullptr);
  auto request = *teaching;
  request.identity.execution_id = execution_id;
  request.identity.supply_epoch = "epoch";
  request.identity.reservation_id = "reservation";
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  motion->fail_motion = "fixture_retreat";
  TaskExecutor executor(motion, tool);
  EXPECT_EQ(executor.execute_grasp_cycle(request, execution_id), Code::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(tool->configured_identity.slot_id, "slot_0");
  EXPECT_EQ(tool->configured_identity.material_id, "Tape17");
  EXPECT_EQ(tool->configured_identity.execution_id, execution_id);
  EXPECT_TRUE(executor.released_at_configured_destination());
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
}

}  // namespace arm_cell_task_execution

namespace arm_cell_task_execution
{
TEST(ConfiguredGraspExecution, RejectsPartialOrMismatchedExecutionIdentityBeforeActuation)
{
  auto motion = std::make_shared<FixtureMotion>();
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  TaskExecutor executor(motion, tool);
  auto request = cycle(GraspStrategy::INTERNAL_RADIAL);
  request.identity.slot_id = "slot_0";
  EXPECT_EQ(executor.execute_grasp_cycle(request, execution_id), Code::TASK_RESULT_INVALID_GOAL);
  request.identity = {"11111111111111111111111111111111", "Tape", "supply", "slot_0", "epoch", "reservation"};
  EXPECT_EQ(executor.execute_grasp_cycle(request, execution_id), Code::TASK_RESULT_INVALID_GOAL);
  EXPECT_TRUE(motion->events.empty());
}
}

namespace arm_cell_task_execution
{
TEST(ConfiguredGraspExecution, PhysicalReleaseRemainsPlacedWhenSceneSynchronizationFails)
{
  auto motion = std::make_shared<FixtureMotion>();
  motion->fail_clear = true;
  auto tool = std::make_shared<FixtureTool>(*motion, GraspStrategy::INTERNAL_RADIAL);
  TaskExecutor executor(motion, tool);
  auto request = cycle(GraspStrategy::INTERNAL_RADIAL);
  request.identity = {execution_id, "Tape", "supply", "slot_0", "epoch", "reservation"};
  EXPECT_THROW(executor.execute_grasp_cycle(request, execution_id), std::runtime_error);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
}
}

TEST(ConfiguredGraspCatalog, SelectsFixedOrSlotSpecificSuctionTeachingWithoutBoreGeometry)
{
  using arm_cell_task_execution::parse_configured_grasp_cycles;
  auto entries = nlohmann::json::parse(R"({"slot_1":{
    "grasp":{"strategy":"surface_suction"},"approach_motion":"above","insertion_motion":"pick",
    "transfer_motion":"place","retreat_motion":"exit","material_id":"part","supply_id":"belt"}})");
  entries["slot_2"] = entries.at("slot_1");
  entries["slot_2"]["approach_motion"] = "above_second_slot";
  const auto slots = parse_configured_grasp_cycles(entries.dump());
  ASSERT_NE(slots.find("slot_1"), nullptr);
  ASSERT_NE(slots.find("slot_2"), nullptr);
  EXPECT_EQ(slots.find("slot_1")->approach_motion, "above");
  EXPECT_EQ(slots.find("slot_2")->approach_motion, "above_second_slot");
  EXPECT_EQ(slots.find("slot_3"), nullptr);

  entries["slot_1"]["pick_slot_id"] = "cell/belt/pick";
  entries["slot_1"]["material_type_ids"] = {"A", "B"};
  EXPECT_THROW(parse_configured_grasp_cycles(entries.dump()), std::invalid_argument);
  entries.erase("slot_2");
  const auto fixed = parse_configured_grasp_cycles(entries.dump());
  const auto * first = fixed.find("cell/belt/pick", "A");
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first, fixed.find("cell/belt/pick", "B"));
  EXPECT_EQ(nullptr, fixed.find("cell/belt/pick", "C"));
  EXPECT_EQ(nullptr, fixed.find("cell/other/pick", "A"));
}
