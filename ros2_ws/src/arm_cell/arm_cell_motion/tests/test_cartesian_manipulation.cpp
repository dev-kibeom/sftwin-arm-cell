#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>

#include "arm_cell_motion/cartesian_manipulation.hpp"
#include "pick_cartesian_diagnostics.hpp"
#include "motion_primitive_policy.hpp"

namespace arm_cell_motion
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

TEST(CartesianManipulationTest, SuccessfulCartesianCalculationRecordsElapsedTime)
{
  CartesianPlanningTimer timer;
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

  ASSERT_TRUE(result.valid);
  const auto elapsed_s = timer.elapsed_s();
  std::ostringstream record;
  record << "{\"status\":\"READY_FOR_EXECUTION\"";
  append_cartesian_planning_elapsed(record, elapsed_s);
  record << '}';
  const auto parsed = nlohmann::json::parse(record.str());

  EXPECT_GT(parsed.at("planning_elapsed_s").get<double>(), 0.0);
  EXPECT_EQ(parsed.at("planning_elapsed_status"), "MEASURED");
}

TEST(CartesianManipulationTest, FailedCartesianCalculationRecordsElapsedTime)
{
  CartesianPlanningTimer timer;
  const auto result = generate(
    [](const auto &, const auto &, auto &, auto & reason) {
      reason = "IK";
      return false;
    },
    [](std::size_t, const auto &, const auto &, auto & reason) {
      reason.clear();
      return true;
    });

  ASSERT_FALSE(result.valid);
  const auto elapsed_s = timer.elapsed_s();
  std::ostringstream record;
  record << "{\"status\":\"FAILED\"";
  append_cartesian_planning_elapsed(record, elapsed_s);
  record << '}';
  const auto parsed = nlohmann::json::parse(record.str());

  EXPECT_GT(parsed.at("planning_elapsed_s").get<double>(), 0.0);
  EXPECT_EQ(parsed.at("planning_elapsed_status"), "MEASURED");
}

TEST(CartesianManipulationTest, PreconditionRejectionRecordsElapsedAsNotStarted)
{
  std::ostringstream record;
  record << "{\"status\":\"REJECTED\"";
  append_cartesian_planning_elapsed(record, std::nullopt);
  record << '}';
  const auto parsed = nlohmann::json::parse(record.str());

  EXPECT_TRUE(parsed.at("planning_elapsed_s").is_null());
  EXPECT_EQ(parsed.at("planning_elapsed_status"), "NOT_STARTED");
}

TEST(CartesianManipulationTest, CartesianPhaseUpdatesPreservePlanningEvidenceAsValidJson)
{
  const auto initial = nlohmann::json{
    {"ik_search_elapsed_s", 0.12},
    {"global_planning", {{"elapsed_s", 0.8}, {"status", "PLANNED_FEASIBLE"}}}}.dump();
  const auto descent = upsert_cartesian_diagnostic(
    initial, "cartesian_descent", "{\"status\":\"READY_FOR_EXECUTION\"}");
  const auto retract = upsert_cartesian_diagnostic(
    descent, "cartesian_retract", "{\"status\":\"FAILED\"}");
  const auto updated_retract = upsert_cartesian_diagnostic(
    retract, "cartesian_retract", "{\"status\":\"READY_FOR_EXECUTION\"}");
  const auto updated_descent = upsert_cartesian_diagnostic(
    updated_retract, "cartesian_descent", "{\"status\":\"READY_FOR_EXECUTION\"}");
  const auto parsed = nlohmann::json::parse(updated_descent);

  EXPECT_DOUBLE_EQ(parsed.at("ik_search_elapsed_s"), 0.12);
  EXPECT_DOUBLE_EQ(parsed.at("global_planning").at("elapsed_s"), 0.8);
  EXPECT_EQ(parsed.at("cartesian_descent").at("status"), "READY_FOR_EXECUTION");
  EXPECT_EQ(parsed.at("cartesian_retract").at("status"), "READY_FOR_EXECUTION");
  EXPECT_EQ(updated_descent.find("\"cartesian_retract\":"),
    updated_descent.rfind("\"cartesian_retract\":"));
  EXPECT_EQ(updated_descent.find("\"cartesian_descent\":"),
    updated_descent.rfind("\"cartesian_descent\":"));
}

TEST(CartesianManipulationTest, GlobalFallbackSuccessPreservesCartesianFailureRecord)
{
  auto failed_cartesian = nlohmann::json{
    {"motion_goal_uuid", "goal-42"}, {"status", "FAILED"},
    {"path_fraction", 0.70},
    {"failure_sample", {{"index", 7}}}};
  failed_cartesian["failure_sample"]["contact_bodies"] = {
    {"available", true},
    {"pairs", nlohmann::json::array({nlohmann::json::array({"finger_link", "raw_part"})})}};
  auto evidence = upsert_cartesian_diagnostic(
    "", "place_cartesian_release", failed_cartesian.dump());

  evidence = upsert_cartesian_ready_diagnostic(
    std::move(evidence), "place_cartesian_release",
    R"({"motion_goal_uuid":"goal-42","status":"READY_FOR_EXECUTION"})",
    "GLOBAL_FALLBACK");
  evidence = upsert_cartesian_diagnostic(
    std::move(evidence), "final_segment_planning",
    R"({"motion_goal_uuid":"goal-42","final_planning_result":"SUCCESS","selected_strategy":"GLOBAL_FALLBACK","global_fallback_attempts":[{"request_index":1,"result":"SUCCESS"}]})");

  const auto parsed = nlohmann::json::parse(evidence);
  const auto & cartesian = parsed.at("place_cartesian_release");
  const auto & final_segment = parsed.at("final_segment_planning");
  EXPECT_EQ(cartesian.at("motion_goal_uuid"), "goal-42");
  EXPECT_EQ(cartesian.at("status"), "FAILED");
  EXPECT_DOUBLE_EQ(cartesian.at("path_fraction").get<double>(), 0.70);
  EXPECT_EQ(cartesian.at("failure_sample").at("index"), 7);
  EXPECT_EQ(cartesian.at("failure_sample").at("contact_bodies").at("pairs").at(0).at(0),
    "finger_link");
  EXPECT_EQ(cartesian.at("failure_sample").at("contact_bodies").at("pairs").at(0).at(1),
    "raw_part");
  EXPECT_EQ(final_segment.at("motion_goal_uuid"), "goal-42");
  EXPECT_EQ(final_segment.at("final_planning_result"), "SUCCESS");
  EXPECT_EQ(final_segment.at("selected_strategy"), "GLOBAL_FALLBACK");
  EXPECT_EQ(final_segment.at("global_fallback_attempts").at(0).at("result"), "SUCCESS");
}

TEST(CartesianManipulationTest, CartesianSelectionRemainsReadyForExecution)
{
  EXPECT_TRUE(cartesian_result_selected_for_execution("CARTESIAN"));
  EXPECT_FALSE(cartesian_result_selected_for_execution("GLOBAL_FALLBACK"));
  const auto ready = upsert_cartesian_ready_diagnostic(
    "", "cartesian_descent",
    R"({"motion_goal_uuid":"goal-43","status":"READY_FOR_EXECUTION"})",
    "CARTESIAN");
  const auto parsed = nlohmann::json::parse(ready);
  EXPECT_EQ(parsed.at("cartesian_descent").at("motion_goal_uuid"), "goal-43");
  EXPECT_EQ(parsed.at("cartesian_descent").at("status"), "READY_FOR_EXECUTION");
  EXPECT_EQ(parsed.at("cartesian_descent").at("selected_strategy"), "CARTESIAN");
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

TEST(CartesianManipulationTest, WorldVerticalPickDescentGuardOnlyAppliesToInsertion)
{
  using TaskType = arm_cell_interfaces::msg::MotionTaskType;
  using Phase = arm_cell_interfaces::msg::MotionTaskPhase;

  EXPECT_TRUE(requires_world_vertical_pick_descent_guard(
      TaskType::TASK_TYPE_PICK, Phase::TASK_PHASE_EXECUTING,
      PickApproachAxis::PLANNING_FRAME_VERTICAL));
  EXPECT_FALSE(requires_world_vertical_pick_descent_guard(
      TaskType::TASK_TYPE_PICK, Phase::TASK_PHASE_RETRACTING,
      PickApproachAxis::PLANNING_FRAME_VERTICAL));
  EXPECT_FALSE(requires_world_vertical_pick_descent_guard(
      TaskType::TASK_TYPE_PICK, Phase::TASK_PHASE_EXECUTING,
      PickApproachAxis::TCP_INSERTION_AXIS));
  EXPECT_FALSE(requires_world_vertical_pick_descent_guard(
      TaskType::TASK_TYPE_PLACE, Phase::TASK_PHASE_EXECUTING,
      PickApproachAxis::PLANNING_FRAME_VERTICAL));
}

TEST(CartesianManipulationTest, VerticalPickDescentPreservesOrientationAndUsesWorldZ)
{
  auto actual_approach = pose(0.32);
  actual_approach.position.x = 0.41;
  actual_approach.position.y = -0.13;
  auto grasp = pose(0.17);
  grasp.position.x = 0.40;
  grasp.position.y = -0.12;
  grasp.orientation.x = 0.2;
  grasp.orientation.w = std::sqrt(1.0 - 0.2 * 0.2);

  geometry_msgs::msg::Pose descent_target;
  ASSERT_TRUE(make_world_vertical_pick_descent_target(
      actual_approach, grasp, descent_target));

  EXPECT_DOUBLE_EQ(descent_target.position.x, actual_approach.position.x);
  EXPECT_DOUBLE_EQ(descent_target.position.y, actual_approach.position.y);
  EXPECT_DOUBLE_EQ(descent_target.position.z, grasp.position.z);
  EXPECT_DOUBLE_EQ(descent_target.orientation.x, grasp.orientation.x);
  EXPECT_DOUBLE_EQ(descent_target.orientation.w, grasp.orientation.w);
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

TEST(CartesianManipulationTest, PreservesFailedCartesianSampleAndContactPairs)
{
  const auto result = generate_cartesian_linear_trajectory(
    pose(0.11), pose(0.06), start_positions(),
    {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"},
    limits(), 0.005,
    [](const auto &, const auto & seed, auto & solution, auto & reason) {
      solution = seed;
      reason.clear();
      return true;
    },
    [](std::size_t index, const auto &, const auto &, auto & reason, auto & contacts) {
      if (index == 5U) {
        reason = "collision/contact";
        contacts = {"arm_link_4", "rawpart_body", "gripper_finger", "rawpart_body"};
        return false;
      }
      reason.clear();
      contacts.clear();
      return true;
    });

  ASSERT_FALSE(result.valid);
  EXPECT_EQ(result.failure.sample_index, 5U);
  EXPECT_EQ(result.failure.reason, "collision/contact");
  EXPECT_EQ(result.failure.contact_bodies,
    (std::vector<std::string>{"arm_link_4", "rawpart_body", "gripper_finger", "rawpart_body"}));
  EXPECT_EQ(result.validated_sample_count, 6U);
  EXPECT_DOUBLE_EQ(result.path_fraction, 0.5);

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "base_link";
  start.pose = pose(0.11);
  geometry_msgs::msg::PoseStamped target = start;
  target.pose = pose(0.06);
  const auto diagnostic = make_pick_cartesian_diagnostic(
    "pick-goal", "cartesian_descent", "FAILED", "collision/contact",
    "tcp_insertion_axis_toward_grasp", start, target, target, result, 0.025);
  const auto parsed = nlohmann::json::parse(diagnostic);

  EXPECT_EQ(parsed.at("motion_goal_uuid"), "pick-goal");
  EXPECT_EQ(parsed.at("stage"), "cartesian_descent");
  EXPECT_EQ(parsed.at("failure_reason"), "collision/contact");
  EXPECT_EQ(parsed.at("path_fraction"), 0.5);
  EXPECT_EQ(parsed.at("sample_count"), 11U);
  EXPECT_EQ(parsed.at("validated_sample_count"), 6U);
  EXPECT_EQ(parsed.at("failure_sample").at("index"), 5U);
  EXPECT_TRUE(parsed.at("failure_sample").at("pose_available").get<bool>());
  ASSERT_TRUE(parsed.at("failure_sample").at("contact_bodies").at("available").get<bool>());
  EXPECT_EQ(parsed.at("failure_sample").at("contact_bodies").at("pairs"),
    nlohmann::json::array({
      nlohmann::json::array({"arm_link_4", "rawpart_body"}),
      nlohmann::json::array({"gripper_finger", "rawpart_body"})}));
  EXPECT_EQ(parsed.at("planning_elapsed_status"), "MEASURED");
}

TEST(CartesianManipulationTest, MarksContactEvidenceUnavailableWhenCollisionCheckWasNotReached)
{
  CartesianLinearGenerationResult result;
  result.valid = false;
  result.reason = "IK";
  result.failure.sample_index = 2U;
  result.failure.reason = "IK";
  result.failure.tcp = pose(0.08);
  result.sample_count = 11U;
  result.validated_sample_count = 2U;
  result.path_fraction = 0.2;
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "base_link";
  start.pose = pose(0.11);
  const auto diagnostic = make_pick_cartesian_diagnostic(
    "pick-goal", "cartesian_descent", "FAILED", "IK",
    "tcp_insertion_axis_toward_grasp", start, start, start, result, 0.01);
  const auto parsed = nlohmann::json::parse(diagnostic);

  EXPECT_FALSE(parsed.at("failure_sample").at("contact_bodies").at("available").get<bool>());
  EXPECT_EQ(
    parsed.at("failure_sample").at("contact_bodies").at("unavailable_reason"),
    "contact_check_not_reached");
  EXPECT_FALSE(parsed.at("failure_sample").at("pose_available").get<bool>());
  EXPECT_EQ(
    parsed.at("failure_sample").at("pose_unavailable_reason"),
    "forward_kinematics_not_captured");
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

}  // namespace arm_cell_motion
