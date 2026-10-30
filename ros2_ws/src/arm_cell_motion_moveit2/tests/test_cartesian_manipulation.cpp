#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include "arm_cell_motion_moveit2/cartesian_manipulation.hpp"
#include "motion_primitive_policy.hpp"

namespace arm_cell_motion_moveit2
{

namespace
{
geometry_msgs::msg::Pose pose(double z)
{
  geometry_msgs::msg::Pose value;
  value.position.z = z;
  value.orientation.w = 1.0;
  return value;
}

std::vector<JointContinuityLimit> limits()
{
  return {
    {-6.2832, 6.2832, true}, {-6.2832, 6.2832, true}, {-2.618, 2.618, false},
    {-6.2832, 6.2832, true}, {-6.2832, 6.2832, true}, {-6.2832, 6.2832, true}};
}

std::vector<double> start_positions()
{
  return {-1.0, 1.0, -1.0, -2.0, 1.0, -4.16802};
}

CartesianLinearGenerationResult generate(
  const CartesianIkSolver & solve_ik,
  const CartesianCollisionChecker & check_collision)
{
  return generate_cartesian_linear_trajectory(
    pose(0.11), pose(0.06), start_positions(),
    {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"},
    limits(), 0.005, solve_ik, check_collision);
}
}

TEST(CartesianManipulationTest, GeneratesStraightFiftyMillimeterInsertion)
{
  std::vector<geometry_msgs::msg::Pose> samples;
  const auto result = generate(
    [](const auto &, const auto & seed, auto & solution, auto & reason) {
      solution = seed;
      reason.clear();
      return true;
    },
    [&samples](std::size_t, const auto & tcp, const auto &, auto & reason) {
      samples.push_back(tcp);
      reason.clear();
      return true;
    });

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.trajectory.points.size(), 11U);
  ASSERT_EQ(samples.size(), 11U);
  EXPECT_NEAR(samples.front().position.z, 0.11, 1e-9);
  EXPECT_NEAR(samples.back().position.z, 0.06, 1e-9);
  for (std::size_t index = 1; index < samples.size(); ++index) {
    EXPECT_NEAR(samples[index].position.x, samples.front().position.x, 1e-9);
    EXPECT_NEAR(samples[index].position.y, samples.front().position.y, 1e-9);
    EXPECT_LT(samples[index].position.z, samples[index - 1].position.z);
  }
  EXPECT_NEAR(result.trajectory.points.back().time_from_start.sec, 0, 1e-9);
  EXPECT_DOUBLE_EQ(result.path_fraction, 1.0);
}

TEST(CartesianManipulationTest, PickAndPlaceManipulationPhasesUseCartesianPrimitive)
{
  using TaskType = arm_cell_interfaces::msg::MotionTaskType;
  using Phase = arm_cell_interfaces::msg::MotionTaskPhase;

  for (const auto task_type : {TaskType::TASK_TYPE_PICK, TaskType::TASK_TYPE_PLACE}) {
    EXPECT_TRUE(
      uses_cartesian_manipulation(
        task_type, Phase::TASK_PHASE_EXECUTING));
    EXPECT_TRUE(
      uses_cartesian_manipulation(
        task_type, Phase::TASK_PHASE_RETRACTING));
    EXPECT_FALSE(
      uses_cartesian_manipulation(
        task_type, Phase::TASK_PHASE_APPROACHING));
  }
  EXPECT_FALSE(
    uses_cartesian_manipulation(
      TaskType::TASK_TYPE_GO_HOME, Phase::TASK_PHASE_EXECUTING));
}

TEST(CartesianManipulationTest, PickAndPlaceInsertionUseValidatedApproachStart)
{
  using TaskType = arm_cell_interfaces::msg::MotionTaskType;
  using Phase = arm_cell_interfaces::msg::MotionTaskPhase;

  EXPECT_TRUE(
    uses_validated_approach_start_state(
      TaskType::TASK_TYPE_PICK, Phase::TASK_PHASE_EXECUTING));
  EXPECT_TRUE(
    uses_validated_approach_start_state(
      TaskType::TASK_TYPE_PLACE, Phase::TASK_PHASE_EXECUTING));
  EXPECT_FALSE(
    uses_validated_approach_start_state(
      TaskType::TASK_TYPE_PICK, Phase::TASK_PHASE_RETRACTING));
  EXPECT_FALSE(
    uses_validated_approach_start_state(
      TaskType::TASK_TYPE_PLACE, Phase::TASK_PHASE_RETRACTING));
}

TEST(CartesianManipulationTest, ApproachAcceptanceBelongsOnlyToPickOrPlaceApproach)
{
  using TaskType = arm_cell_interfaces::msg::MotionTaskType;
  using Phase = arm_cell_interfaces::msg::MotionTaskPhase;
  EXPECT_TRUE(requires_approach_entry_acceptance(
      TaskType::TASK_TYPE_PICK, Phase::TASK_PHASE_APPROACHING));
  EXPECT_TRUE(requires_approach_entry_acceptance(
      TaskType::TASK_TYPE_PLACE, Phase::TASK_PHASE_APPROACHING));
  EXPECT_FALSE(requires_approach_entry_acceptance(
      TaskType::TASK_TYPE_GO_HOME, Phase::TASK_PHASE_APPROACHING));
  EXPECT_FALSE(requires_approach_entry_acceptance(
      TaskType::TASK_TYPE_RETRACT, Phase::TASK_PHASE_APPROACHING));
  EXPECT_FALSE(requires_approach_entry_acceptance(
      TaskType::TASK_TYPE_PICK, Phase::TASK_PHASE_RETRACTING));
}

TEST(CartesianManipulationTest, PlaceInsertionAndRetractKeepRecipeOrientation)
{
  using TaskType = arm_cell_interfaces::msg::MotionTaskType;
  constexpr double half_angle = 0.37;
  const auto target_orientation = Eigen::Quaterniond(
    std::cos(half_angle), 0.0, 0.0, std::sin(half_angle));
  geometry_msgs::msg::Pose target;
  target.position.x = 0.500;
  target.position.y = 0.250;
  target.position.z = 0.3675;
  target.orientation.z = target_orientation.z();
  target.orientation.w = target_orientation.w();

  for (const auto & endpoints : {
      std::pair{0.4175, 0.3675}, std::pair{0.3675, 0.4175}})
  {
    auto segment_target = target;
    segment_target.position.z = endpoints.second;
    auto measured_start = segment_target;
    measured_start.position.z = endpoints.first;
    measured_start.orientation.w = 1.0;
    measured_start.orientation.z = 0.0;
    const auto path_start = cartesian_start_pose(
      measured_start, segment_target, TaskType::TASK_TYPE_PLACE);
    EXPECT_DOUBLE_EQ(path_start.position.z, endpoints.first);
    EXPECT_DOUBLE_EQ(path_start.orientation.z, target.orientation.z);
    EXPECT_DOUBLE_EQ(path_start.orientation.w, target.orientation.w);

    std::vector<geometry_msgs::msg::Pose> samples;
    const auto result = generate_cartesian_linear_trajectory(
      path_start, segment_target, start_positions(),
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"},
      limits(), 0.005,
      [](const auto &, const auto & seed, auto & solution, auto & reason) {
        solution = seed;
        reason.clear();
        return true;
      },
      [&samples](std::size_t, const auto & tcp, const auto &, auto & reason) {
        samples.push_back(tcp);
        reason.clear();
        return true;
      });

    ASSERT_TRUE(result.valid) << result.reason;
    ASSERT_EQ(samples.size(), 11U);
    EXPECT_DOUBLE_EQ(samples.front().position.x, 0.500);
    EXPECT_DOUBLE_EQ(samples.front().position.y, 0.250);
    EXPECT_DOUBLE_EQ(samples.front().position.z, endpoints.first);
    EXPECT_DOUBLE_EQ(samples.back().position.z, endpoints.second);
    for (const auto & sample : samples) {
      EXPECT_NEAR(sample.orientation.z, target.orientation.z, 1e-12);
      EXPECT_NEAR(sample.orientation.w, target.orientation.w, 1e-12);
    }
  }
}

TEST(CartesianManipulationTest, PickInsertionUsesResolvedTargetOrientation)
{
  using TaskType = arm_cell_interfaces::msg::MotionTaskType;
  auto measured_start = pose(0.11);
  measured_start.orientation.x = 0.2;
  measured_start.orientation.w = std::sqrt(1.0 - 0.2 * 0.2);
  const auto target = pose(0.06);

  const auto path_start = cartesian_start_pose(
    measured_start, target, TaskType::TASK_TYPE_PICK);

  EXPECT_DOUBLE_EQ(path_start.orientation.x, target.orientation.x);
  EXPECT_DOUBLE_EQ(path_start.orientation.w, target.orientation.w);
}

TEST(CartesianManipulationTest, SeedsEachIKCallWithPreviousAcceptedJointState)
{
  std::vector<std::vector<double>> seeds;
  const auto result = generate(
    [&seeds](const auto &, const auto & seed, auto & solution, auto & reason) {
      seeds.push_back(seed);
      solution = seed;
      solution[0] += 0.001;
      reason.clear();
      return true;
    },
    [](std::size_t, const auto &, const auto &, auto & reason) {
      reason.clear();
      return true;
    });

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(seeds.size(), 10U);
  EXPECT_NEAR(seeds[0][0], start_positions()[0], 1e-9);
  EXPECT_NEAR(seeds[1][0], result.trajectory.points[1].positions[0], 1e-9);
  EXPECT_NEAR(
    seeds.back()[0],
    result.trajectory.points[result.trajectory.points.size() - 2].positions[0], 1e-9);
}

TEST(CartesianManipulationTest, RejectsIntermediateCollision)
{
  const auto result = generate(
    [](const auto &, const auto & seed, auto & solution, auto & reason) {
      solution = seed;
      reason.clear();
      return true;
    },
    [](std::size_t index, const auto &, const auto &, auto & reason) {
      if (index == 5U) {
        reason = "collision/contact";
        return false;
      }
      reason.clear();
      return true;
    });

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure.sample_index, 5U);
  EXPECT_EQ(result.failure.reason, "collision/contact");
}

TEST(CartesianManipulationTest, RejectsIntermediateIkFailure)
{
  const auto result = generate(
    [](const auto &, const auto & seed, auto & solution, auto & reason) {
      if (seed[0] < -0.995) {
        reason = "IK";
        return false;
      }
      solution = seed;
      reason.clear();
      return true;
    },
    [](std::size_t, const auto &, const auto &, auto & reason) {
      reason.clear();
      return true;
    });

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure.sample_index, 1U);
  EXPECT_EQ(result.failure.reason, "IK");
}

TEST(CartesianManipulationTest, RejectsAccumulatedWrapDiscontinuity)
{
  const auto result = generate(
    [](const auto & tcp, const auto & seed, auto & solution, auto & reason) {
      solution = seed;
      const double progress = (0.11 - tcp.position.z) / 0.05;
      solution[5] = -4.16802 + 6.28381 * progress;
      reason.clear();
      return true;
    },
    [](std::size_t, const auto &, const auto &, auto & reason) {
      reason.clear();
      return true;
    });

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure.reason, "continuity");
}

TEST(CartesianManipulationTest, ProducesCompleteTrajectoryForTiming)
{
  const auto result = generate(
    [](const auto &, const auto & seed, auto & solution, auto & reason) {
      solution = seed;
      reason.clear();
      return true;
    },
    [](std::size_t, const auto &, const auto &, auto & reason) {
      reason.clear();
      return true;
    });

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.trajectory.joint_names.size(), 6U);
  ASSERT_EQ(result.trajectory.points.size(), 11U);
  for (const auto & point : result.trajectory.points) {
    ASSERT_EQ(point.positions.size(), 6U);
    EXPECT_TRUE(point.velocities.empty());
    EXPECT_TRUE(point.accelerations.empty());
  }
}

TEST(CartesianManipulationTest, AcceptsValidPostTimingTrajectoryAsExecutionAuthority)
{
  const auto generated = generate(
    [](const auto &, const auto & seed, auto & solution, auto & reason) {
      solution = seed;
      reason.clear();
      return true;
    },
    [](std::size_t, const auto &, const auto &, auto & reason) {
      reason.clear();
      return true;
    });
  ASSERT_TRUE(generated.valid);

  auto timed_trajectory = generated.trajectory;
  for (std::size_t index = 0; index < timed_trajectory.points.size(); ++index) {
    timed_trajectory.points[index].time_from_start.sec = static_cast<int32_t>(index);
  }
  std::vector<std::vector<double>> validated_states;
  const auto validation = validate_cartesian_joint_trajectory(
    timed_trajectory, start_positions(), limits(),
    [&validated_states](std::size_t, const auto & positions, auto & reason, auto & contacts) {
      validated_states.push_back(positions);
      reason.clear();
      contacts.clear();
      return true;
    });

  ASSERT_TRUE(validation.valid) << validation.reason;
  ASSERT_EQ(validated_states.size(), timed_trajectory.points.size());
  for (std::size_t index = 0; index < validated_states.size(); ++index) {
    EXPECT_EQ(validated_states[index], timed_trajectory.points[index].positions);
  }
}

TEST(CartesianManipulationTest, RejectsPostTimingContinuityViolation)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint_6"};
  trajectory.points.resize(3);
  trajectory.points[0].positions = {-4.16802};
  trajectory.points[1].positions = {-4.1};
  trajectory.points[2].positions = {2.11579};

  const auto validation = validate_cartesian_joint_trajectory(
    trajectory, {-4.16802}, {{-6.2832, 6.2832, true}},
    [](std::size_t, const auto &, auto & reason, auto & contacts) {
      reason.clear();
      contacts.clear();
      return true;
    });

  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.failure.sample_index, 2U);
  EXPECT_EQ(validation.failure.reason, "continuity");
}

TEST(CartesianManipulationTest, RejectsPostTimingCollisionInvalidState)
{
  const auto generated = generate(
    [](const auto &, const auto & seed, auto & solution, auto & reason) {
      solution = seed;
      reason.clear();
      return true;
    },
    [](std::size_t, const auto &, const auto &, auto & reason) {
      reason.clear();
      return true;
    });
  ASSERT_TRUE(generated.valid);

  const auto validation = validate_cartesian_joint_trajectory(
    generated.trajectory, start_positions(), limits(),
    [](std::size_t index, const auto &, auto & reason, auto & contacts) {
      if (index == 4U) {
        reason = "collision/contact";
        contacts = {"link_4", "cube_profile"};
        return false;
      }
      reason.clear();
      contacts.clear();
      return true;
    });

  EXPECT_FALSE(validation.valid);
  EXPECT_EQ(validation.failure.sample_index, 4U);
  EXPECT_EQ(validation.failure.reason, "collision/contact");
  EXPECT_EQ(
    validation.failure.contact_bodies,
    std::vector<std::string>({"link_4", "cube_profile"}));
}

}  // namespace arm_cell_motion_moveit2
