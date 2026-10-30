#include <gtest/gtest.h>

#include <cstddef>

#include "arm_cell_motion/pick_search_budget.hpp"
#include "arm_cell_motion/pick_search_controller.hpp"

namespace arm_cell_motion
{
namespace
{
using Evaluation = PickSearchEvaluation;
using Status = PickSearchStatus;

TEST(PickSearchController, FirstExecutableApproachStopsWithoutAnotherAttempt)
{
  PickSearchController search(48U, 2U);
  ASSERT_EQ(search.next(100.0), Status::SEARCHING);
  EXPECT_EQ(search.pose_index(), 0U);
  EXPECT_EQ(search.seed_attempt(), 1U);
  EXPECT_EQ(search.seed_source(), PickSearchSeed::MEASURED);
  search.evaluate(Evaluation::EXECUTABLE_APPROACH_SELECTED);
  EXPECT_EQ(search.next(99.0), Status::SELECTED);
  EXPECT_EQ(search.attempt_count(), 1U);
}

TEST(PickSearchController, RetriesSeedThenAdvancesPose)
{
  PickSearchController search(2U, 2U);
  ASSERT_EQ(search.next(100.0), Status::SEARCHING);
  search.evaluate(Evaluation::GRASP_VALIDATION_FAILED);
  ASSERT_EQ(search.next(99.0), Status::SEARCHING);
  EXPECT_EQ(search.pose_index(), 0U);
  EXPECT_EQ(search.seed_attempt(), 2U);
  EXPECT_EQ(search.seed_source(), PickSearchSeed::DIVERSIFIED);
  search.evaluate(Evaluation::GRASP_VALIDATION_FAILED);
  ASSERT_EQ(search.next(98.0), Status::SEARCHING);
  EXPECT_EQ(search.pose_index(), 1U);
  EXPECT_EQ(search.seed_attempt(), 1U);
}

TEST(PickSearchController, PlanningFailureSkipsRemainingSeed)
{
  PickSearchController search(2U, 2U);
  ASSERT_EQ(search.next(100.0), Status::SEARCHING);
  search.evaluate(Evaluation::GLOBAL_PLANNING_FAILED);
  ASSERT_EQ(search.next(99.0), Status::SEARCHING);
  EXPECT_EQ(search.pose_index(), 1U);
  EXPECT_EQ(search.seed_attempt(), 1U);
}

TEST(PickSearchController, ExhaustsAllPosesAndPreservesResumeFloor)
{
  PickSearchController search(3U, 2U, 1U);
  for (std::size_t pose = 1U; pose < 3U; ++pose) {
    for (std::size_t seed = 1U; seed <= 2U; ++seed) {
      ASSERT_EQ(search.next(100.0), Status::SEARCHING);
      EXPECT_EQ(search.pose_index(), pose);
      EXPECT_EQ(search.seed_attempt(), seed);
      search.evaluate(Evaluation::GRASP_VALIDATION_FAILED);
    }
  }
  EXPECT_EQ(search.next(100.0), Status::ALL_CANDIDATES_EXHAUSTED);
  EXPECT_EQ(search.attempt_count(), 4U);
  PickSearchController empty(0U, 2U);
  EXPECT_EQ(empty.next(100.0), Status::ALL_CANDIDATES_EXHAUSTED);
}

TEST(PickSearchController, SharedBudgetIsNotResetByPoseTransition)
{
  const auto start = PickSearchBudget::Clock::time_point{};
  const PickSearchBudget budget(100.0, start);
  PickSearchController search(48U, 2U);
  ASSERT_EQ(search.next(budget.remaining_s(start)), Status::SEARCHING);
  search.evaluate(Evaluation::GLOBAL_PLANNING_FAILED);
  const auto deadline = start + std::chrono::seconds(100);
  EXPECT_EQ(search.next(budget.remaining_s(deadline)), Status::SEARCH_BUDGET_EXHAUSTED);
  EXPECT_EQ(search.attempt_count(), 1U);
  EXPECT_EQ(budget.call_timeout_s(1.0, deadline), 0.0);
}

TEST(PickSearchController, InterruptionPreemptsBudgetAndDiscardsInFlightResult)
{
  for (const auto evaluation : {Evaluation::GRASP_VALIDATION_FAILED,
      Evaluation::GLOBAL_PLANNING_FAILED, Evaluation::EXECUTABLE_APPROACH_SELECTED})
  {
    PickSearchController search(48U, 2U);
    ASSERT_EQ(search.next(100.0), Status::SEARCHING);
    search.evaluate(evaluation);
    EXPECT_EQ(search.next(0.0, true), Status::INTERRUPTED);
    search.evaluate(Evaluation::EXECUTABLE_APPROACH_SELECTED);
    EXPECT_EQ(search.next(100.0), Status::INTERRUPTED);
    EXPECT_EQ(search.attempt_count(), 1U);
  }
}

TEST(PickSearchController, NoSolutionVisitsEveryMeasuredSeedBeforeDeferredSeeds)
{
  for (int repeat = 0; repeat < 2; ++repeat) {
    PickSearchController search(3U, 2U);
    for (std::size_t seed = 1U; seed <= 2U; ++seed) {
      for (std::size_t pose = 0U; pose < 3U; ++pose) {
        ASSERT_EQ(search.next(100.0), Status::SEARCHING);
        EXPECT_EQ(search.pose_index(), pose);
        EXPECT_EQ(search.seed_attempt(), seed);
        search.evaluate(Evaluation::GRASP_IK_NO_SOLUTION);
      }
    }
    EXPECT_EQ(search.next(100.0), Status::ALL_CANDIDATES_EXHAUSTED);
    EXPECT_EQ(search.attempt_count(), 6U);
  }
}

TEST(PickSearchController, LaterMeasuredSuccessPreemptsDeferredSeed)
{
  PickSearchController search(3U, 2U);
  ASSERT_EQ(search.next(100.0), Status::SEARCHING);
  search.evaluate(Evaluation::GRASP_IK_NO_SOLUTION);
  ASSERT_EQ(search.next(99.0), Status::SEARCHING);
  EXPECT_EQ(search.pose_index(), 1U);
  EXPECT_EQ(search.seed_source(), PickSearchSeed::MEASURED);
  search.evaluate(Evaluation::EXECUTABLE_APPROACH_SELECTED);
  EXPECT_EQ(search.next(98.0), Status::SELECTED);
  EXPECT_EQ(search.attempt_count(), 2U);
}

TEST(PickSearchController, DeferredPassRespectsResumeFloorAndSinglePose)
{
  for (const auto count : {1U, 4U}) {
    const std::size_t floor = count == 1U ? 0U : 2U;
    PickSearchController search(count, 2U, floor);
    for (std::size_t seed = 1U; seed <= 2U; ++seed) {
      for (std::size_t pose = floor; pose < count; ++pose) {
        ASSERT_EQ(search.next(100.0), Status::SEARCHING);
        EXPECT_EQ(search.pose_index(), pose);
        EXPECT_EQ(search.seed_attempt(), seed);
        search.evaluate(Evaluation::GRASP_IK_NO_SOLUTION);
      }
    }
    EXPECT_EQ(search.next(100.0), Status::ALL_CANDIDATES_EXHAUSTED);
    EXPECT_EQ(search.attempt_count(), (count - floor) * 2U);
  }
}

TEST(PickSearchController, JointConfigurationFailuresRetryImmediatelyAndNeverRevisit)
{
  for (const auto failure : {Evaluation::GRASP_COLLISION, Evaluation::APPROACH_IK_FAILED,
      Evaluation::APPROACH_VALIDATION_FAILED, Evaluation::GRASP_VALIDATION_FAILED,
      Evaluation::GRASP_IK_FAILED})
  {
    PickSearchController search(3U, 2U);
    for (const auto pose : {0U, 0U, 1U, 2U, 1U}) {
      ASSERT_EQ(search.next(100.0), Status::SEARCHING);
      EXPECT_EQ(search.pose_index(), pose);
      search.evaluate(
        pose == 0U ? failure : pose == 1U ?
        Evaluation::GRASP_IK_NO_SOLUTION : Evaluation::GLOBAL_PLANNING_FAILED);
    }
    EXPECT_EQ(search.next(100.0), Status::ALL_CANDIDATES_EXHAUSTED);
    EXPECT_EQ(search.attempt_count(), 5U);
  }
}

TEST(PickSearchController, ContactAndPlanningFailureSkipUnusedSeedWithoutRejectingOtherPoses)
{
  for (const auto failure : {Evaluation::GRASP_CONTACT_MISMATCH,
      Evaluation::GLOBAL_PLANNING_FAILED})
  {
    PickSearchController search(2U, 2U);
    ASSERT_EQ(search.next(100.0), Status::SEARCHING);
    search.evaluate(failure);
    ASSERT_EQ(search.next(99.0), Status::SEARCHING);
    EXPECT_EQ(search.pose_index(), 1U);
    EXPECT_EQ(search.seed_source(), PickSearchSeed::MEASURED);
    search.evaluate(Evaluation::GRASP_IK_NO_SOLUTION);
    ASSERT_EQ(search.next(98.0), Status::SEARCHING);
    EXPECT_EQ(search.pose_index(), 1U);
    EXPECT_EQ(search.seed_source(), PickSearchSeed::DIVERSIFIED);
    search.evaluate(failure);
    EXPECT_EQ(search.next(97.0), Status::ALL_CANDIDATES_EXHAUSTED);
  }
}

TEST(PickSearchController, DeferredSuccessStopsAtFirstExecutableApproach)
{
  PickSearchController search(2U, 2U);
  for (int pose = 0; pose < 2; ++pose) {
    ASSERT_EQ(search.next(100.0), Status::SEARCHING);
    search.evaluate(Evaluation::GRASP_IK_NO_SOLUTION);
  }
  ASSERT_EQ(search.next(98.0), Status::SEARCHING);
  EXPECT_EQ(search.pose_index(), 0U);
  EXPECT_EQ(search.seed_attempt(), 2U);
  search.evaluate(Evaluation::EXECUTABLE_APPROACH_SELECTED);
  EXPECT_EQ(search.next(97.0), Status::SELECTED);
  EXPECT_EQ(search.attempt_count(), 3U);
}

TEST(PickSearchController, DeferredPassSharesDeadlineAndStopsWithPendingPoses)
{
  const auto start = PickSearchBudget::Clock::time_point{};
  const PickSearchBudget budget(100.0, start);
  for (const auto elapsed : {99, 100}) {
    PickSearchController search(2U, 2U);
    for (int pose = 0; pose < 2; ++pose) {
      ASSERT_EQ(search.next(budget.remaining_s(start)), Status::SEARCHING);
      search.evaluate(Evaluation::GRASP_IK_NO_SOLUTION);
    }
    const auto status = search.next(budget.remaining_s(start + std::chrono::seconds(elapsed)));
    EXPECT_EQ(status, elapsed == 99 ? Status::SEARCHING : Status::SEARCH_BUDGET_EXHAUSTED);
    if (elapsed == 99) {
      EXPECT_EQ(search.pose_index(), 0U);
      search.evaluate(Evaluation::GRASP_IK_NO_SOLUTION);
      EXPECT_EQ(
        search.next(budget.remaining_s(start + std::chrono::seconds(100))),
        Status::SEARCH_BUDGET_EXHAUSTED);
    }
    EXPECT_EQ(search.attempt_count(), elapsed == 99 ? 3U : 2U);
  }
}

TEST(PickSearchController, BackendErrorStopsWithDeferredPosesPending)
{
  PickSearchController search(2U, 2U);
  ASSERT_EQ(search.next(100.0), Status::SEARCHING);
  search.evaluate(Evaluation::GRASP_IK_NO_SOLUTION);
  ASSERT_EQ(search.next(99.0), Status::SEARCHING);
  search.evaluate(Evaluation::BACKEND_ERROR);
  EXPECT_EQ(search.next(98.0), Status::BACKEND_ERROR);
  search.evaluate(Evaluation::GRASP_COLLISION);
  EXPECT_EQ(search.next(97.0), Status::BACKEND_ERROR);
  EXPECT_EQ(search.attempt_count(), 2U);
}
}  // namespace
}  // namespace arm_cell_motion

TEST(PickSearchBudget, CandidateDeadlineDoesNotResetActionDeadline)
{
  using Budget = arm_cell_motion::PickSearchBudget;
  const auto start = Budget::Clock::time_point{};
  const auto second = start + std::chrono::seconds(14);
  const Budget first(100.0, start, 15.0, start);
  EXPECT_DOUBLE_EQ(first.remaining_s(second), 1.0);
  EXPECT_DOUBLE_EQ(first.remaining_s(start + std::chrono::seconds(16)), 0.0);
  const Budget next(100.0, start, 15.0, second);
  EXPECT_DOUBLE_EQ(next.remaining_s(second), 15.0);
  const auto late = start + std::chrono::seconds(95);
  const Budget last(100.0, start, 15.0, late);
  EXPECT_DOUBLE_EQ(last.call_timeout_s(15.0, late), 5.0);
}
