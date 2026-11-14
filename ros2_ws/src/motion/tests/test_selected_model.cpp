#include <gtest/gtest.h>
#include "arm_cell_motion/grasp_collision_validator.hpp"
#include "arm_cell_motion/pick_grasp_pose_candidates.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <algorithm>
#include <fstream>
#include <moveit_msgs/msg/collision_object.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sstream>

// Input comes from Bringup's real composition/export, expanded by Xacro.
TEST(SelectedModel, LoadsSolverAndValidatesConfiguredPoses)
{
  const auto * input = std::getenv("ARM_CELL_MODEL_TEST_INPUT");
  ASSERT_NE(input, nullptr);
  nlohmann::json data;
  std::ifstream(input) >> data;
  const auto & config = data["configuration"];
  std::ifstream semantic_file(
    config["model_configuration"]["semantic"].get<std::string>());
  std::stringstream semantic;
  semantic << semantic_file.rdbuf();
  rclcpp::init(0, nullptr);
  std::vector<rclcpp::Parameter> parameters{
    {"robot_description", data["urdf"].get<std::string>()},
    {"robot_description_semantic", semantic.str()}};
  const auto group_name =
    config["model_configuration"]["planning_group"].get<std::string>();
  for (const auto & item : data["kinematics"][group_name].items()) {
    const auto name =
      "robot_description_kinematics." + group_name + "." + item.key();
    if (item.value().is_string()) {
      parameters.emplace_back(name, item.value().get<std::string>());
    } else {
      parameters.emplace_back(name, item.value().get<double>());
    }
  }
  auto node = std::make_shared<rclcpp::Node>(
    "selected_model_test",
    rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .parameter_overrides(parameters));
  robot_model_loader::RobotModelLoader loader(node);
  auto model = loader.getModel();
  ASSERT_NE(model, nullptr);
  const auto * group = model->getJointModelGroup(
    config["model_configuration"]["planning_group"]);
  ASSERT_NE(group, nullptr);
  EXPECT_EQ(
    group->getVariableNames(),
    config["motion"]["arm_joint_names"].get<std::vector<std::string>>());
  ASSERT_NE(group->getSolverInstance(), nullptr);
  for (const auto * link : model->getLinkModels()) {
    const auto urdf_link = model->getURDF()->getLink(link->getName());
    if (urdf_link && !urdf_link->collision_array.empty()) {
      ASSERT_FALSE(link->getShapes().empty())
        << "Collision mesh did not load: " << link->getName();
    }
  }
  planning_scene::PlanningScene scene(model);
  auto & state = scene.getCurrentStateNonConst();
  for (const auto & pose :
    {config["home_joint_positions"],
      config["motion"]["retract_joint_positions"],
      config["motion"]["place_tray_return_joint_positions"]})
  {
    state.setToDefaultValues();
    state.setJointGroupPositions(group, pose.get<std::vector<double>>());
    state.update();
    EXPECT_TRUE(state.satisfiesBounds(group));
    collision_detection::CollisionRequest request;
    request.contacts = true;
    collision_detection::CollisionResult result;
    scene.checkSelfCollision(request, result, state);
    for (const auto & contact : result.contacts) {
      std::cerr << "configured pose collision: " << contact.first.first << " / "
                << contact.first.second << '\n';
    }
    EXPECT_FALSE(result.collision);
    // Exercise the configured solver on a pose produced by this model's FK.
    const auto target = state.getGlobalLinkTransform("sf_grasp_tcp");
    EXPECT_TRUE(state.setFromIK(group, target, "sf_grasp_tcp", 0.1));
    // Real meshes expose inner-knuckle interference at the TestCube center.
    // Keep that rejection; a shallower offset may provide clearance.
    const Eigen::Quaterniond object_orientation(
      target.linear() * Eigen::AngleAxisd(
        3.14159265358979323846,
        Eigen::Vector3d::UnitX()).toRotationMatrix());
    const auto grasp = arm_cell_motion::validate_grasp_collision(
      arm_cell_motion::GraspOccupancyMode::NONE, scene, scene, state, target,
      target.translation(), object_orientation,
      arm_cell_motion::BoxGraspContact{0U, 70.0}, Eigen::Vector3d::UnitZ(), {},
      arm_cell_motion::FingerClearanceParameters{}, group_name,
      Eigen::Vector3d(0.07, 0.07, 0.08));
    for (const auto & contact : grasp.common.contacts) {
      std::cerr << "center grasp collision: " << contact.first << " / " << contact.second << '\n';
    }
    EXPECT_TRUE(grasp.valid);
    EXPECT_FALSE(grasp.allowed);
    EXPECT_TRUE(
      std::any_of(
        grasp.common.contacts.begin(), grasp.common.contacts.end(),
        [](const auto & contact) {
          return (contact.first == "pick_selected_target" &&
          contact.second.find("inner_knuckle_link") != std::string::npos) ||
          (contact.second == "pick_selected_target" &&
          contact.first.find("inner_knuckle_link") != std::string::npos);
        }));
    bool shallow_offset_allowed = false;
    arm_cell_motion::PickGraspSearchSampling sampling;
    const auto & motion = config["motion"];
    sampling.maximum_withdrawal_tilt_deg = motion["pick_max_withdrawal_tilt_deg"];
    sampling.position_range_half_length_fraction =
      motion["pick_position_range_half_length_fraction"];
    sampling.position_offset_samples_per_tangent_axis =
      motion["pick_position_offset_samples_per_tangent_axis"];
    sampling.orientation_samples_per_closing_axis =
      motion["pick_orientation_samples_per_closing_axis"];
    sampling.candidate_limit = motion["pick_grasp_pose_candidate_limit"];
    sampling.maximum_downward_position_offset_m = motion["pick_max_downward_position_offset_m"];
    const auto candidates = arm_cell_motion::generate_pick_grasp_search_candidates(
      Eigen::Quaterniond::Identity(), Eigen::Quaterniond(
        Eigen::AngleAxisd(
          3.14159265358979323846, Eigen::Vector3d::UnitX())),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
      0.07, 0.07, 0.08, 85.0, sampling);
    for (const auto & candidate : candidates) {
      if (candidate.grasp_position_offset_object.z() <= 0.0) {continue;}
      // Relocate the target by the inverse selected Object-to-TCP pose.
      // This checks exact sampled tool/object geometry without claiming that
      // the arm can reach every sample from these configured joint poses.
      Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
      relative.linear() = candidate.tcp_orientation.toRotationMatrix();
      relative.translation() = candidate.grasp_position_offset_object;
      const Eigen::Isometry3d shifted_object = target * relative.inverse();
      const auto shifted = arm_cell_motion::validate_grasp_collision(
        arm_cell_motion::GraspOccupancyMode::NONE, scene, scene, state, target,
        shifted_object.translation(), Eigen::Quaterniond(shifted_object.linear()),
        candidate.contact, Eigen::Vector3d::UnitZ(), {},
        arm_cell_motion::FingerClearanceParameters{}, group_name,
        Eigen::Vector3d(0.07, 0.07, 0.08));
      if (shifted.allowed) {
        std::cerr << "clear sampled offset: " << candidate.grasp_position_offset_object.transpose()
                  << " width_mm=" << candidate.contact->width_mm << '\n';
        shallow_offset_allowed = true;
        break;
      }
    }
    EXPECT_TRUE(shallow_offset_allowed);
  }
  // Load the selected Cell's actual static geometry in the canonical base
  // frame.
  nlohmann::json geometry;
  std::ifstream(config["cell"]["static_scene"].get<std::string>()) >> geometry;
  const auto & base = config["cell"]["world_to_base"];
  Eigen::Isometry3d world_from_base = Eigen::Isometry3d::Identity();
  world_from_base.translation() =
    Eigen::Vector3d(base["x"], base["y"], base["z"]);
  world_from_base.linear() =
    (Eigen::AngleAxisd(base["yaw"], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(base["pitch"], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(base["roll"], Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
  for (const auto & object : geometry["objects"]) {
    moveit_msgs::msg::CollisionObject collision;
    collision.header.frame_id = "base_link";
    collision.id = object["id"];
    shape_msgs::msg::SolidPrimitive box;
    ASSERT_EQ(object["geometry"]["type"], "box");
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    const auto dimensions =
      object["geometry"]["dimensions_m"].get<std::vector<double>>();
    box.dimensions.assign(dimensions.begin(), dimensions.end());
    collision.primitives.push_back(box);
    const auto & p = object["pose"]["position_m"];
    const auto & q = object["pose"]["quaternion_xyzw"];
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(p[0], p[1], p[2]);
    pose.linear() =
      Eigen::Quaterniond(q[3], q[0], q[1], q[2]).toRotationMatrix();
    pose = world_from_base.inverse() * pose;
    geometry_msgs::msg::Pose message;
    message.position.x = pose.translation().x();
    message.position.y = pose.translation().y();
    message.position.z = pose.translation().z();
    const Eigen::Quaterniond rotation(pose.linear());
    message.orientation.x = rotation.x();
    message.orientation.y = rotation.y();
    message.orientation.z = rotation.z();
    message.orientation.w = rotation.w();
    collision.primitive_poses.push_back(message);
    collision.operation = moveit_msgs::msg::CollisionObject::ADD;
    ASSERT_TRUE(scene.processCollisionObjectMsg(collision));
  }
  state.setToDefaultValues();
  state.setJointGroupPositions(
    group, config["home_joint_positions"].get<std::vector<double>>());
  state.update();
  collision_detection::CollisionRequest cell_request;
  cell_request.contacts = true;
  cell_request.group_name = group->getName();
  collision_detection::CollisionResult cell_result;
  scene.checkCollision(cell_request, cell_result, state);
  for (const auto & contact : cell_result.contacts) {
    std::cerr << "HOME Cell collision: " << contact.first.first << " / "
              << contact.first.second << '\n';
  }
  EXPECT_FALSE(cell_result.collision) << "HOME intersects selected Cell";
  // Representative static reachability only: Tray centre and configured CNC
  // object pose. This does not assert a collision-free approach or a complete
  // PnP trajectory.
  auto transform =
    [](const nlohmann::json & position, const nlohmann::json & orientation)
    {
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
      result.translation() =
        Eigen::Vector3d(position[0], position[1], position[2]);
      result.linear() = Eigen::Quaterniond(
        orientation[3], orientation[0],
        orientation[1], orientation[2])
        .toRotationMatrix();
      return result;
    };
  const auto object_to_tcp =
    transform(
    config["motion"]["object_to_grasp_tcp_translation"],
    config["motion"]["object_to_grasp_tcp_quaternion"]);
  std::vector<std::pair<std::string, Eigen::Isometry3d>> targets;
  const auto & cnc = config["cell"]["work_locations"]["cnc_loading"];
  targets.emplace_back(
    "CNC",
    transform(cnc["position"], cnc["orientation_xyzw"]) * object_to_tcp);
  nlohmann::json full_geometry;
  std::ifstream(config["cell"]["geometry"].get<std::string>()) >> full_geometry;
  for (const auto & object : full_geometry["objects"]) {
    if (object["id"] !=
      config["cell"]["material_supply"]["support_geometry_object"])
    {
      continue;
    }
    auto tray_pose = transform(
      object["pose"]["position_m"],
      object["pose"]["quaternion_xyzw"]);
    const auto material_id =
      config["orchestration"]["mission_target_id"].get<std::string>();
    const double height =
      config["material_definitions"][material_id]["nominal_dimensions_m"][2];
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation().z() =
      object["geometry"]["dimensions_m"][2].get<double>() / 2 + height / 2;
    targets.emplace_back(
      "Tray centre", world_from_base.inverse() * tray_pose *
      offset * object_to_tcp);
  }
  for (const auto & target : targets) {
    bool reachable = false;
    const auto seeds = config["robot"]["joint_policy"]["place_ik_seed_joints"]
      .get<std::vector<std::string>>();
    std::size_t count = 1;
    for (std::size_t index = 0; index < seeds.size(); ++index) {
      count *= 3;
    }
    for (std::size_t trial = 0; trial < count && !reachable; ++trial) {
      state.setToDefaultValues();
      state.setJointGroupPositions(
        group, config["home_joint_positions"].get<std::vector<double>>());
      auto code = trial;
      for (const auto & seed : seeds) {
        const auto & bounds = model->getVariableBounds(seed);
        const double fraction = static_cast<int>(code % 3) - 1;
        code /= 3;
        state.setVariablePosition(
          seed,
          (bounds.min_position_ + bounds.max_position_) / 2 +
          fraction * (bounds.max_position_ - bounds.min_position_) / 4);
      }
      state.update();
      if (!state.setFromIK(group, target.second, "sf_grasp_tcp", 0.1)) {continue;}
      if (!state.satisfiesBounds(group)) {continue;}
      state.update();
      reachable = !scene.isStateColliding(state, group->getName());
    }
    EXPECT_TRUE(reachable)
      << target.first << " has no collision-free static IK in selected Cell";
    std::cout << "Static IK " << target.first << ": " << reachable << '\n';
  }
  rclcpp::shutdown();
}
