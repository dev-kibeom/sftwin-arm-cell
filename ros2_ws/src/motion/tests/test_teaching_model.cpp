#include <gtest/gtest.h>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "arm_cell_motion/taught_motion.hpp"
#include "arm_cell_motion/taught_motion_trajectory.hpp"
#include "arm_cell_motion/trajectory_generation.hpp"


TEST(TeachingModel, GeneratesCompleteConfiguredTapeMotionsWithoutSceneOrPlanner)
{
  using namespace arm_cell_motion;
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  const auto * input = std::getenv("ARM_CELL_MODEL_TEST_INPUT");
  ASSERT_NE(input, nullptr);
  nlohmann::json data;
  std::ifstream(input) >> data;
  const auto & config = data.at("configuration");
  const auto & model_config = config.at("model_configuration");
  const auto group_name = model_config.at("planning_group").get<std::string>();
  std::ifstream semantic_file(model_config.at("semantic").get<std::string>());
  std::stringstream semantic; semantic << semantic_file.rdbuf();
  rclcpp::init(0, nullptr);
  std::vector<rclcpp::Parameter> parameters{
    {"robot_description", data.at("urdf").get<std::string>()},
    {"robot_description_semantic", semantic.str()}};
  for (const auto & item : data.at("kinematics").at(group_name).items()) {
    const auto key = "robot_description_kinematics." + group_name + "." + item.key();
    if (item.value().is_string()) {parameters.emplace_back(key, item.value().get<std::string>());}
    else {parameters.emplace_back(key, item.value().get<double>());}
  }
  const auto & motion = config.at("motion");
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "robot_description_kinematics." + group_name + ".kinematics_solver") {
      parameter = rclcpp::Parameter(parameter.get_name(), motion.at("pick_ik_solver").get<std::string>());
    }
  }
  parameters.emplace_back("robot_description_kinematics." + group_name + ".solve_type",
    motion.at("pick_ik_solve_type").get<std::string>());
  auto node = std::make_shared<rclcpp::Node>("teaching_model_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    .parameter_overrides(parameters));
  robot_model_loader::RobotModelLoader loader(node);
  auto model = loader.getModel();
  ASSERT_NE(model, nullptr);
  const auto * group = model->getJointModelGroup(group_name);
  ASSERT_NE(group, nullptr);
  ASSERT_NE(group->getSolverInstance(), nullptr);
  auto source_solver = std::const_pointer_cast<kinematics::KinematicsBase>(group->getSolverInstance());
  const auto timeout = source_solver->getDefaultTimeout();
  source_solver->setDefaultTimeout(.37);
  model = make_kinematic_model(model, group_name, source_solver);
  ASSERT_DOUBLE_EQ(source_solver->getDefaultTimeout(), .37)
    << "Teaching model must not change another consumer's solver timeout";
  source_solver->setDefaultTimeout(timeout);
  group = model->getJointModelGroup(group_name);
  const auto names = motion.at("arm_joint_names").get<std::vector<std::string>>();
  const auto limits = joint_motion_limits_from_urdf(data.at("urdf").get<std::string>(), names,
    motion.at("planning_max_acceleration_rad_s2").get<double>());
  const auto registry = parse_taught_motions(motion.at("taught_motions_json").get<std::string>(),
    config.at("home_joint_positions").get<std::vector<double>>(),
    config.at("home_joint_positions").get<std::vector<double>>(), names.size());
  moveit::core::RobotState state(model); state.setToDefaultValues();
  state.setJointGroupPositions(group, config.at("home_joint_positions").get<std::vector<double>>());
  state.update();
  for (const auto * sequence : {"tape_approach", "tape_insert", "tape_transfer", "tape_retreat"}) {
    const auto & taught = registry.at(sequence);
    for (const auto & point : taught.cartesian_waypoints) {
      SCOPED_TRACE(std::string(sequence) + ":" + point.motion);
      Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
      target.linear() = Eigen::Quaterniond(point.pose[6], point.pose[3], point.pose[4], point.pose[5]).toRotationMatrix();
      target.translation() = Eigen::Vector3d(point.pose[0], point.pose[1], point.pose[2]);
      target = state.getGlobalLinkTransform("base_link") * target;
      trajectory_msgs::msg::JointTrajectory trajectory;
      if (point.motion == "MoveJ") {
        std::vector<double> joints, start;
        ASSERT_EQ(resolve_pose_joint_target(state, group_name, "sf_grasp_tcp", target, joints), Code::TASK_RESULT_SUCCESS);
        state.copyJointGroupPositions(group, start);
        const auto generated = make_taught_joint_trajectory(names, start, joints, limits,
          taught.velocity_scale, taught.acceleration_scale);
        ASSERT_TRUE(generated.valid) << generated.reason;
        trajectory = generated.trajectory;
      } else {
        ASSERT_EQ(point.motion, "MoveL");
        moveit_msgs::msg::RobotTrajectory generated;
        ASSERT_EQ(generate_linear_joint_trajectory(state, group_name, "sf_grasp_tcp", target, generated), Code::TASK_RESULT_SUCCESS);
        ASSERT_TRUE(retime_joint_trajectory(model, group_name, state, generated,
          taught.velocity_scale, taught.acceleration_scale, &limits));
        trajectory = generated.joint_trajectory;
      }
      ASSERT_GE(trajectory.points.size(), 2U);
      std::cout << sequence << " " << point.motion << " target_joints="
                << nlohmann::json(trajectory.points.back().positions).dump() << '\n';
      state.setJointGroupPositions(group, trajectory.points.back().positions); state.update();
      EXPECT_TRUE(state.satisfiesBounds(group));
      const auto actual = state.getGlobalLinkTransform("sf_grasp_tcp");
      EXPECT_LT((actual.translation() - target.translation()).norm(), 1e-4);
      EXPECT_LT(Eigen::AngleAxisd(actual.linear().transpose() * target.linear()).angle(), 1e-3);
      for (const auto & sample : trajectory.points) {
        for (std::size_t j = 0; j < limits.size(); ++j) {
          EXPECT_LE(std::abs(sample.velocities[j]), limits[j].max_velocity * taught.velocity_scale + 1e-6);
          EXPECT_LE(std::abs(sample.accelerations[j]), limits[j].max_acceleration * taught.acceleration_scale + 1e-6);
        }
      }
    }
  }
  rclcpp::shutdown();
}
