#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include "arm_cell_motion/pick_planning_diagnostics.hpp"

namespace arm_cell_motion
{
TEST(PickPlanningDiagnostics, PreservesAttemptEvidenceWithoutChangingExecutionState)
{
  PickPlanningDiagnostics diagnostics;
  diagnostics.resize(2U);
  PickPlanningAttempt attempt;
  attempt.current_pose_index = 1U;
  attempt.seed_index = 1U;
  attempt.attempt_evaluation = PickSearchEvaluation::GRASP_COLLISION;
  attempt.failure_stage = "GRASP_COLLISION";
  attempt.failure_reason = "collision/contact";
  attempt.current_seed = {0.0};
  attempt.grasp_joint_positions = {0.1};
  attempt.executed_stages = {"SolveGraspIK", "ValidateGraspKinematics", "ValidateGraspCollision"};
  attempt.stage_elapsed_s = {{"SolveGraspIK", 0.25}, {"ValidateGraspKinematics", 0.1},
    {"ValidateGraspCollision", 0.05}};
  attempt.grasp_collision_result.collision = true;
  const auto result = PickPlanningResult();
  const PickPlanningDiagnosticMetadata metadata{1U, 2U, {"SolveGraspIK"},
    {{"link_1", "fixture", "grasp_target_state", false, 0U, 2U, 1U}}};
  diagnostics.collect(attempt, result, metadata);
  const auto & summary = diagnostics.summary();
  ASSERT_EQ(summary.pick_attempt_records.size(), 1U);
  const auto record = nlohmann::json::parse(summary.pick_attempt_records.front());
  EXPECT_EQ(record.at("search_evaluation"), "GRASP_COLLISION");
  EXPECT_DOUBLE_EQ(record.at("ik_elapsed_s").get<double>(), 0.25);
  EXPECT_EQ(record.at("approach_ik").at("calls"), 0);
  EXPECT_EQ(record.at("approach_ik").at("result"), "NOT_RUN");
  EXPECT_EQ(summary.candidate_results[1], "REJECTED");
  EXPECT_EQ(summary.candidate_transition_reasons[1], "collision/contact");
  EXPECT_EQ(summary.collision_rejected, 1U);
  ASSERT_EQ(summary.all_pick_collision_contacts.size(), 1U);
  EXPECT_EQ(summary.all_pick_collision_contacts.front().body_2, "fixture");
  EXPECT_EQ(attempt.attempt_evaluation, PickSearchEvaluation::GRASP_COLLISION);
  EXPECT_EQ(attempt.executed_stages.size(), 3U);
  EXPECT_FALSE(result.selected);
  EXPECT_EQ(result.search_status, PickSearchStatus::SEARCHING);
}

TEST(PickPlanningDiagnostics, BudgetAndCancelDoNotInventSkippedStageSuccess)
{
  for (const auto * stage : {"SEARCH_BUDGET", "INTERRUPTED"}) {
    PickPlanningDiagnostics diagnostics;
    diagnostics.resize(1U);
    PickPlanningAttempt attempt;
    attempt.seed_index = 1U;
    attempt.failure_stage = stage;
    attempt.failure_reason = "RESULT_DISCARDED";
    diagnostics.collect(
      attempt,
      PickPlanningResult(), PickPlanningDiagnosticMetadata{1U, 2U, {}, {}});
    const auto record = nlohmann::json::parse(diagnostics.summary().pick_attempt_records.front());
    EXPECT_FALSE(record.at("grasp_ik").at("called").get<bool>());
    EXPECT_EQ(record.at("grasp_validation"), "NOT_RUN");
    EXPECT_EQ(record.at("approach_ik").at("result"), "NOT_RUN");
    EXPECT_EQ(record.at("planning"), "NOT_RUN");
    EXPECT_TRUE(record.at("executed_stages").empty());
    EXPECT_EQ(diagnostics.summary().planning_attempted, 0U);
  }
}
}  // namespace arm_cell_motion
