#include <gtest/gtest.h>

#include "arm_cell_motion_moveit2/final_target_hold.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{
TEST(FinalTargetHold, CommandsOriginalTargetWithZeroVelocity)
{
  FinalTargetHold hold;
  ASSERT_TRUE(hold.begin({1.0, -2.0}, 10.0));

  const auto command = hold.command(10.5);
  EXPECT_EQ(command.positions, (std::vector<double>{1.0, -2.0}));
  EXPECT_EQ(command.velocities, (std::vector<double>{0.0, 0.0}));
  EXPECT_DOUBLE_EQ(command.trajectory_time, 0.5);
}

TEST(FinalTargetHold, RepeatedCommandsDoNotSubstituteMeasuredPosition)
{
  FinalTargetHold hold;
  ASSERT_TRUE(hold.begin({1.0, -2.0}, 10.0));

  const auto first = hold.command(10.1);
  const auto second = hold.command(11.1);
  EXPECT_EQ(first.positions, second.positions);
  EXPECT_EQ(second.positions, (std::vector<double>{1.0, -2.0}));
}

TEST(FinalTargetHold, CancelSuppressesFutureCommands)
{
  FinalTargetHold hold;
  ASSERT_TRUE(hold.begin({1.0}, 10.0));
  hold.cancel();

  EXPECT_FALSE(hold.active());
  EXPECT_TRUE(hold.command(10.1).positions.empty());
}
}  // namespace
}  // namespace arm_cell_motion_moveit2
