#include <gtest/gtest.h>

#include "arm_cell_motion/pick_grasp_contact.hpp"

namespace arm_cell_motion
{
namespace
{
TEST(PickGraspContactTest, ReportsContactAndMismatchEvidenceIndependently)
{
  const auto accepted = validate_pick_grasp_contact(
    Eigen::Quaterniond::Identity(), std::size_t{0}, Eigen::Quaterniond::Identity(),
    0.07, 0.05, 0.08, 40.0, 85.0, 85.0);
  EXPECT_EQ(accepted.check, PickGraspContactCheck::VALID);
  ASSERT_TRUE(accepted.contact);
  EXPECT_EQ(accepted.contact->closing_axis, 0U);
  EXPECT_TRUE(accepted.reason.empty());

  const auto mismatch = validate_pick_grasp_contact(
    Eigen::Quaterniond::Identity(), std::size_t{1}, Eigen::Quaterniond::Identity(),
    0.07, 0.05, 0.08, 40.0, 85.0, 85.0);
  EXPECT_EQ(mismatch.check, PickGraspContactCheck::MISMATCH);
  ASSERT_TRUE(mismatch.contact);
  EXPECT_EQ(mismatch.contact->closing_axis, 0U);
  EXPECT_EQ(mismatch.reason, "grasp_contact_mismatch");

  const auto not_required = validate_pick_grasp_contact(
    Eigen::Quaterniond::Identity(), std::nullopt, Eigen::Quaterniond::Identity(),
    0.07, 0.05, 0.08, 40.0, 85.0, 85.0);
  EXPECT_EQ(not_required.check, PickGraspContactCheck::NOT_REQUIRED);
  EXPECT_FALSE(not_required.contact);
  EXPECT_TRUE(not_required.reason.empty());
}
}  // namespace
}  // namespace arm_cell_motion
