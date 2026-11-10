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

TEST(PickSearchController, EveryPoseIsVisitedBeforeAnotherSeedForEachRetryableFailure)
{
  for (const auto failure : {Evaluation::GRASP_IK_NO_SOLUTION, Evaluation::GRASP_COLLISION,
      Evaluation::APPROACH_IK_FAILED, Evaluation::GRASP_VALIDATION_FAILED,
      Evaluation::GRASP_IK_FAILED, Evaluation::APPROACH_VALIDATION_FAILED})
  {
    for (const auto limit : {1U, 2U, 3U}) {
      PickSearchController search(48U, limit);
      PickSearchBudget budget;
      const auto later = PickSearchBudget::Clock::now() + std::chrono::hours(1);
      for (std::size_t seed = 1U; seed <= limit; ++seed) {
        for (std::size_t pose = 0U; pose < 48U; ++pose) {
          ASSERT_EQ(search.next(budget.remaining_s(later)), Status::SEARCHING);
          EXPECT_EQ(search.pose_index(), pose);
          EXPECT_EQ(search.seed_attempt(), seed);
          EXPECT_EQ(
            search.seed_source(), seed == 1 ?
            PickSearchSeed::MEASURED : PickSearchSeed::DIVERSIFIED);
          search.evaluate(failure);
        }
      }
      EXPECT_EQ(search.next(budget.remaining_s(later)), Status::ALL_CANDIDATES_EXHAUSTED);
      EXPECT_EQ(search.attempt_count(), 48U * limit);
      EXPECT_DOUBLE_EQ(budget.call_timeout_s(1.0, later), 1.0);
    }
  }
}

TEST(PickSearchController, LaterPoseSuccessStopsImmediately)
{
  PickSearchController search(48U, 2U);
  PickSearchBudget budget;
  for (std::size_t pose = 0; pose < 48U; ++pose) {
    ASSERT_EQ(search.next(budget.remaining_s()), Status::SEARCHING);
    search.evaluate(
      pose == 47U ? Evaluation::EXECUTABLE_APPROACH_SELECTED :
      Evaluation::GRASP_IK_NO_SOLUTION);
  }
  EXPECT_EQ(search.next(budget.remaining_s()), Status::SELECTED);
  EXPECT_EQ(search.attempt_count(), 48U);
}

TEST(PickSearchController, IneligibleFailuresSkipAllRemainingSeeds)
{
  for (const auto failure : {Evaluation::GRASP_CONTACT_MISMATCH,
      Evaluation::GLOBAL_PLANNING_FAILED})
  {
    PickSearchController search(3U, 3U);
    for (std::size_t pose = 0U; pose < 3U; ++pose) {
      ASSERT_EQ(search.next(1.0), Status::SEARCHING);
      EXPECT_EQ(search.pose_index(), pose);
      search.evaluate(failure);
    }
    EXPECT_EQ(search.next(1.0), Status::ALL_CANDIDATES_EXHAUSTED);
    EXPECT_EQ(search.attempt_count(), 3U);
  }
}

TEST(PickSearchController, ResumeFloorAndMixedFailuresRemainBounded)
{
  PickSearchController search(4U, 3U, 2U);
  for (const auto pose : {2U, 3U, 3U, 3U}) {
    ASSERT_EQ(search.next(1.0), Status::SEARCHING);
    EXPECT_EQ(search.pose_index(), pose);
    search.evaluate(
      pose == 2U ? Evaluation::GRASP_CONTACT_MISMATCH :
      Evaluation::GRASP_COLLISION);
  }
  EXPECT_EQ(search.next(1.0), Status::ALL_CANDIDATES_EXHAUSTED);
  PickSearchController empty(0U, 2U);
  EXPECT_EQ(empty.next(1.0), Status::ALL_CANDIDATES_EXHAUSTED);
}

TEST(PickSearchController, InterruptionAndBackendFailureStopPendingRounds)
{
  PickSearchController search(48U, 2U);
  ASSERT_EQ(search.next(1.0), Status::SEARCHING);
  search.evaluate(Evaluation::GRASP_COLLISION);
  EXPECT_EQ(search.next(1.0, true), Status::INTERRUPTED);
  search.evaluate(Evaluation::EXECUTABLE_APPROACH_SELECTED);
  EXPECT_EQ(search.next(1.0), Status::INTERRUPTED);
  PickSearchController backend(48U, 2U);
  ASSERT_EQ(backend.next(1.0), Status::SEARCHING);
  backend.evaluate(Evaluation::BACKEND_ERROR);
  EXPECT_EQ(backend.next(1.0), Status::BACKEND_ERROR);
}

TEST(PickSearchController, PreferredPosesUseBothSeedsBeforeFallbackExpandsSearch)
{
  PickSearchController search(4U, 2U, 0U, 2U);
  for (const auto pose : {0U, 1U, 0U, 1U, 2U, 3U, 2U, 3U}) {
    ASSERT_EQ(search.next(10.0), PickSearchStatus::SEARCHING);
    EXPECT_EQ(search.pose_index(), pose);
    search.evaluate(PickSearchEvaluation::GRASP_IK_NO_SOLUTION);
  }
  EXPECT_EQ(search.next(10.0), PickSearchStatus::ALL_CANDIDATES_EXHAUSTED);
}
}  // namespace
}  // namespace arm_cell_motion
