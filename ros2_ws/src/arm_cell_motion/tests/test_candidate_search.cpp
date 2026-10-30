#include <gtest/gtest.h>

#include "arm_cell_motion/candidate_search.hpp"

namespace arm_cell_motion
{
namespace
{

TEST(CandidateSearch, DistanceFirstPreservesPickAxisHeightAndIdOrdering)
{
  const std::vector<CandidatePriority> input{
    {0U, 1.0, 0.2, "z"}, {1U, 2.0, 0.1, "b"},
    {2U, 2.0, 0.3, "c"}, {3U, 2.0, 0.3, "a"}};
  EXPECT_EQ(
    order_candidate_indices(input, CandidatePriorityPolicy::DISTANCE_FIRST),
    (std::vector<std::size_t>{3U, 2U, 1U, 0U}));
}

TEST(CandidateSearch, HeightFirstKeepsVisionOrderForEqualHeights)
{
  const std::vector<CandidatePriority> input{
    {0U, 0.0, 0.2, "z"}, {1U, 99.0, 0.3, "b"},
    {2U, -99.0, 0.3, "a"}, {3U, 0.0, 0.1, "c"}};
  EXPECT_EQ(
    order_candidate_indices(input, CandidatePriorityPolicy::HEIGHT_FIRST),
    (std::vector<std::size_t>{1U, 2U, 0U, 3U}));
}

TEST(CandidateSearch, ReturnsIndicesIntoTheOriginalCandidateList)
{
  const std::vector<CandidatePriority> input{
    {7U, 0.0, 0.2, "lower"}, {3U, 0.0, 0.4, "higher"}};
  EXPECT_EQ(
    order_candidate_indices(input, CandidatePriorityPolicy::HEIGHT_FIRST),
    (std::vector<std::size_t>{3U, 7U}));
}

TEST(CandidateSearch, CursorAdvancesOnlyWhenCallerRequestsNextCandidate)
{
  CandidateSearchCursor cursor;
  cursor.reset({4U, 2U});
  std::size_t selected = 0U;
  ASSERT_TRUE(cursor.select_next(selected));
  EXPECT_EQ(selected, 4U);
  EXPECT_EQ(cursor.selected_count(), 1U);
  ASSERT_TRUE(cursor.select_next(selected));
  EXPECT_EQ(selected, 2U);
  EXPECT_FALSE(cursor.select_next(selected));
}

}  // namespace
}  // namespace arm_cell_motion
