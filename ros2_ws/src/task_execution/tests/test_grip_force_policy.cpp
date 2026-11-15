#include "arm_cell_motion/fake_motion_backend.hpp"
#include <gtest/gtest.h>
#include <limits>
#include "arm_cell_task_execution/grip_force_policy.hpp"

using arm_cell_task_execution::GripForcePolicy;

TEST(GripForcePolicyTest, UsesMassFrictionAccelerationAndSafetyFactor)
{
  GripForcePolicy policy;
  policy.materials.emplace("steel", arm_cell_task_execution::MaterialGripProperties{2., .5, 235.});
  EXPECT_NEAR(policy.required_force("steel"), 86.4532, 1e-6);
  policy.acceleration_bound_m_s2 = 3.;
  EXPECT_NEAR(policy.required_force("steel"), 102.4532, 1e-6);
}

TEST(GripForcePolicyTest, ManufacturerMinimumMustFitMaterialLimit)
{
  GripForcePolicy policy;
  policy.materials.emplace("light", arm_cell_task_execution::MaterialGripProperties{.1, .5, 50.});
  EXPECT_DOUBLE_EQ(policy.required_force("light"), 40.);
  policy.materials.at("light").damage_max_force_n = 39.;
  EXPECT_THROW(policy.required_force("light"), std::invalid_argument);
}

TEST(GripForcePolicyTest, RejectsInfeasibleForceWithoutClamping)
{
  GripForcePolicy policy;
  policy.materials.emplace("heavy", arm_cell_task_execution::MaterialGripProperties{20., .5, 1000.});
  EXPECT_THROW(policy.required_force("heavy"), std::invalid_argument);
  policy.materials.at("heavy") = {2., .5, 80.};
  EXPECT_THROW(policy.required_force("heavy"), std::invalid_argument);
  EXPECT_THROW(policy.required_force("unknown"), std::invalid_argument);
}

TEST(GripForcePolicyTest, RejectsInvalidInputsAndAcceptsFrictionAboveOne)
{
  GripForcePolicy policy;
  policy.materials.emplace("rubber", arm_cell_task_execution::MaterialGripProperties{2., 1.2, 235.});
  EXPECT_DOUBLE_EQ(policy.required_force("rubber"), 40.);
  policy.materials.at("rubber").friction_coefficient_lower_bound = 0.;
  EXPECT_THROW(policy.required_force("rubber"), std::invalid_argument);
  policy.materials.at("rubber").friction_coefficient_lower_bound = .5;
  policy.safety_factor = 1.;
  EXPECT_THROW(policy.required_force("rubber"), std::invalid_argument);
  policy.safety_factor = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(policy.required_force("rubber"), std::invalid_argument);
}

TEST(GripForcePolicyTest, FakeBackendAcceptsForceAwareClose)
{
  arm_cell_motion::FakeMotionBackend backend;
  auto port = backend.gripper_port();
  EXPECT_EQ(port->close(40.0), arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(port->holding().state, arm_cell_motion::HoldingState::HELD);
  EXPECT_EQ(port->close(-1.0), arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
}
