#include <array>
#include <vector>

#include <gtest/gtest.h>

#include "arm_cell_motion/disturbance_motion.hpp"
#include "arm_cell_motion/candidate_search.hpp"

namespace
{
geometry_msgs::msg::PoseStamped representative(double height)
{
  geometry_msgs::msg::PoseStamped value;
  value.header.frame_id = "base_link";
  value.pose.position.x = 0.1;
  value.pose.position.y = 0.2;
  value.pose.position.z = height;
  return value;
}
}

TEST(DisturbanceMotion, UsesSharedHeightPriorityAndPreservesEnumerationForTies)
{
  const std::vector<arm_cell_motion::CandidatePriority> priorities{
    {0, 0.0, 0.1, "low"}, {1, 0.0, 0.3, "tie-first"},
    {2, 0.0, 0.8, "high"}, {3, 0.0, 0.3, "tie-second"}};

  const auto ranked = arm_cell_motion::order_candidate_indices(
    priorities, arm_cell_motion::CandidatePriorityPolicy::HEIGHT_FIRST);

  ASSERT_EQ(ranked.size(), 4U);
  EXPECT_EQ(ranked, (std::vector<std::size_t>{2U, 1U, 3U, 0U}));
}

TEST(DisturbanceMotion, BuildsShortFourDirectionRoutesFromRepresentativePoint)
{
  const auto input = representative(0.4);
  constexpr double push_m = 0.02;
  constexpr double approach_m = 0.01;
  const std::array<arm_cell_motion::DisturbanceDirection, 4> directions{
    arm_cell_motion::DisturbanceDirection::POSITIVE_X,
    arm_cell_motion::DisturbanceDirection::NEGATIVE_X,
    arm_cell_motion::DisturbanceDirection::POSITIVE_Y,
    arm_cell_motion::DisturbanceDirection::NEGATIVE_Y};

  for (const auto direction : directions) {
    const auto route = arm_cell_motion::make_disturbance_route(
      input, direction, push_m, approach_m);
    EXPECT_NEAR(route.approach.pose.position.z, 0.4, 1e-12);
    EXPECT_NEAR(route.push_end.pose.position.z, 0.4, 1e-12);
    EXPECT_NEAR(route.retract_end.pose.position.z, 0.4, 1e-12);
    if (direction == arm_cell_motion::DisturbanceDirection::POSITIVE_X) {
      EXPECT_DOUBLE_EQ(route.approach.pose.position.x, 0.09);
      EXPECT_DOUBLE_EQ(route.push_end.pose.position.x, 0.12);
      EXPECT_DOUBLE_EQ(route.retract_end.pose.position.x, 0.13);
    }
  }
}
