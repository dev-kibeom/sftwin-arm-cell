#include <limits>
#include <gtest/gtest.h>
#include "arm_cell_sim_adapter/canonical_joint_state.hpp"

using arm_cell_sim_adapter::canonical_joint_state;

TEST(CanonicalJointState, ProjectsMeasuredValuesByNameAndPreservesStamp)
{
  sensor_msgs::msg::JointState raw;
  raw.header.stamp.sec = 12;
  raw.header.stamp.nanosec = 345;
  raw.name = {"mimic", "joint_2", "joint_1"};
  raw.position = {9.0, 2.0, 1.0};
  raw.velocity = {8.0, 0.2, 0.1};
  raw.effort = {7.0, 20.0, 10.0};
  auto result = canonical_joint_state(raw, {"joint_1", "joint_2"});
  ASSERT_TRUE(result);
  EXPECT_EQ(result->header.stamp, raw.header.stamp);
  EXPECT_EQ(result->name, (std::vector<std::string>{"joint_1", "joint_2"}));
  EXPECT_EQ(result->position, (std::vector<double>{1.0, 2.0}));
  EXPECT_EQ(result->velocity, (std::vector<double>{0.1, 0.2}));
  EXPECT_EQ(result->effort, (std::vector<double>{10.0, 20.0}));
}

TEST(CanonicalJointState, RejectsMissingDuplicateAndMalformedSamples)
{
  sensor_msgs::msg::JointState raw;
  raw.name = {"joint_1"};
  raw.position = {1.0};
  EXPECT_FALSE(canonical_joint_state(raw, {"joint_1", "joint_2"}));
  raw.name = {"joint_1", "joint_1"};
  raw.position = {1.0, 2.0};
  EXPECT_FALSE(canonical_joint_state(raw, {"joint_1"}));
  raw.name = {"joint_1"};
  EXPECT_FALSE(canonical_joint_state(raw, {"joint_1"}));
  raw.position = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(canonical_joint_state(raw, {"joint_1"}));
  raw.position = {1.0};
  raw.velocity = {1.0, 2.0};
  EXPECT_FALSE(canonical_joint_state(raw, {"joint_1"}));
  raw.velocity = {std::numeric_limits<double>::infinity()};
  EXPECT_FALSE(canonical_joint_state(raw, {"joint_1"}));
}

TEST(CanonicalJointState, AllowsAbsentOptionalFields)
{
  sensor_msgs::msg::JointState raw;
  raw.name = {"joint_1"};
  raw.position = {0.3};
  auto result = canonical_joint_state(raw, {"joint_1"});
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->velocity.empty());
  EXPECT_TRUE(result->effort.empty());
}
