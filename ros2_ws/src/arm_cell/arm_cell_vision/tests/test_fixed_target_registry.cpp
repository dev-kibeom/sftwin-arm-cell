#include <cmath>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "arm_cell_vision/fixed_target_registry.hpp"

namespace
{

arm_cell_vision::FixedTargetRegistration valid_registration()
{
  arm_cell_vision::FixedTargetRegistration registration;
  registration.target_id = "cube_profile";
  registration.run_id = "run-1";
  registration.pose.header.frame_id = "base_link";
  registration.pose.pose.position.x = 0.2;
  registration.pose.pose.position.y = -0.1;
  registration.pose.pose.position.z = 0.35;
  registration.pose.pose.orientation.w = 1.0;
  registration.has_target_yaw = true;
  registration.ttl_ns = 30'000'000'000LL;
  return registration;
}

arm_cell_vision::FixedTargetRegistry make_registry(int64_t * now)
{
  return arm_cell_vision::FixedTargetRegistry(
    "cube_profile",
    [now]() {return *now;});
}

TEST(FixedTargetRegistry, RegistrationDefaultsToThirtySecondTtl)
{
  const arm_cell_vision::FixedTargetRegistration registration;
  EXPECT_EQ(registration.ttl_ns, 30'000'000'000LL);
}

TEST(FixedTargetRegistry, AcceptsValidRegistrationAndConsumesItOnce)
{
  int64_t now = 100;
  auto registry = make_registry(&now);

  const auto accepted = registry.register_target(valid_registration());
  ASSERT_TRUE(accepted.accepted);
  EXPECT_EQ(accepted.receipt_sequence, 1U);
  EXPECT_EQ(accepted.receipt_time_ns, now);

  const auto result = registry.consume("cube_profile", now + 1);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->run_id, "run-1");
  EXPECT_TRUE(result->has_target_yaw);
  EXPECT_FALSE(registry.consume("cube_profile", now + 2).has_value());
}

TEST(FixedTargetRegistry, RejectsWrongFrameNonFiniteAndZeroNormOrientation)
{
  int64_t now = 100;
  auto registry = make_registry(&now);

  auto wrong_frame = valid_registration();
  wrong_frame.pose.header.frame_id = "camera_link";
  EXPECT_FALSE(registry.register_target(wrong_frame).accepted);

  auto non_finite = valid_registration();
  non_finite.pose.pose.position.x = NAN;
  EXPECT_FALSE(registry.register_target(non_finite).accepted);

  auto zero_orientation = valid_registration();
  zero_orientation.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(registry.register_target(zero_orientation).accepted);
}

TEST(FixedTargetRegistry, RejectsUnknownTargetAndInvalidTtl)
{
  int64_t now = 100;
  auto registry = make_registry(&now);

  auto unknown = valid_registration();
  unknown.target_id = "other";
  EXPECT_FALSE(registry.register_target(unknown).accepted);

  auto zero_ttl = valid_registration();
  zero_ttl.ttl_ns = 0;
  EXPECT_FALSE(registry.register_target(zero_ttl).accepted);

  auto over_cap = valid_registration();
  over_cap.ttl_ns = 60'000'000'001LL;
  EXPECT_FALSE(registry.register_target(over_cap).accepted);
}

TEST(FixedTargetRegistry, SameRunReplacesUnconsumedRecord)
{
  int64_t now = 100;
  auto registry = make_registry(&now);

  ASSERT_TRUE(registry.register_target(valid_registration()).accepted);
  auto replacement = valid_registration();
  replacement.pose.pose.position.x = 0.8;
  ASSERT_TRUE(registry.register_target(replacement).accepted);

  const auto result = registry.consume("cube_profile", now + 1);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->pose.pose.position.x, 0.8);
  EXPECT_EQ(result->receipt_sequence, 2U);
}

TEST(FixedTargetRegistry, DifferentRunCannotReplaceUntilConsumedOrExpired)
{
  int64_t now = 100;
  auto registry = make_registry(&now);

  ASSERT_TRUE(registry.register_target(valid_registration()).accepted);
  auto other_run = valid_registration();
  other_run.run_id = "run-2";
  EXPECT_FALSE(registry.register_target(other_run).accepted);

  ASSERT_TRUE(registry.consume("cube_profile", now + 1).has_value());
  EXPECT_TRUE(registry.register_target(other_run).accepted);
}

TEST(FixedTargetRegistry, ExpiredRegistrationIsFailClosedAndAllowsNextRun)
{
  int64_t now = 100;
  auto registry = make_registry(&now);

  auto registration = valid_registration();
  registration.ttl_ns = 10;
  ASSERT_TRUE(registry.register_target(registration).accepted);
  now = 111;
  EXPECT_FALSE(registry.consume("cube_profile", now).has_value());

  auto next_run = valid_registration();
  next_run.run_id = "run-2";
  EXPECT_TRUE(registry.register_target(next_run).accepted);
}

}  // namespace
