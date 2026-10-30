#include "arm_cell_motion/motion_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arm_cell_interfaces/msg/stop_mode.hpp>
#include <arm_cell_interfaces/srv/stop_motion.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "arm_cell_motion/isaac_motion_backend.hpp"
#include "arm_cell_motion/isaac_ros_motion_transport.hpp"
#include "arm_cell_motion/pick_grasp_pose_candidates.hpp"
#include "arm_cell_motion/grasp_collision_validator.hpp"

namespace arm_cell_motion
{

namespace
{
MotionGeometry read_motion_geometry(rclcpp::Node & node)
{
  const auto observation_translation = node.declare_parameter<std::vector<double>>(
    "observation_to_object_translation", std::vector<double>{});
  const auto observation_rotation = node.declare_parameter<std::vector<double>>(
    "observation_to_object_quaternion", std::vector<double>{});
  const auto observation_reference = node.declare_parameter<std::string>(
    "observation_reference", "");
  const auto translation = node.declare_parameter<std::vector<double>>(
    "object_to_grasp_tcp_translation", std::vector<double>{});
  const auto rotation = node.declare_parameter<std::vector<double>>(
    "object_to_grasp_tcp_quaternion", std::vector<double>{});
  const auto insertion_axis = node.declare_parameter<std::vector<double>>(
    "insertion_axis_tcp", std::vector<double>{});
  const auto pick_retract_distance = node.declare_parameter<double>(
    "pick_retract_distance_m", -1.0);
  const auto place_waypoint_clearance_boundary = node.declare_parameter<double>(
    "place_waypoint_clearance_y_boundary_m", std::numeric_limits<double>::quiet_NaN());
  const auto place_waypoint_clearance_y_margin = node.declare_parameter<double>(
    "place_waypoint_clearance_y_margin_m", 0.05);
  const auto place_waypoint_clearance_direction = node.declare_parameter<std::string>(
    "place_waypoint_clearance_direction", "");
  const auto place_waypoint_approach_distance = node.declare_parameter<double>(
    "place_waypoint_approach_distance_m", std::numeric_limits<double>::quiet_NaN());
  const auto place_waypoint_target = node.declare_parameter<std::vector<double>>(
    "place_waypoint_target_pose_xyzw", std::vector<double>{});
  const auto place_waypoint_direction = node.declare_parameter<std::vector<double>>(
    "place_waypoint_approach_direction_object", std::vector<double>{});

  MotionGeometry geometry;
  geometry.observation_reference = observation_reference;
  geometry.pick_retract_distance_m = pick_retract_distance;
  if (std::isfinite(place_waypoint_clearance_boundary) &&
    place_waypoint_clearance_boundary < 0.0 &&
    place_waypoint_clearance_direction == "POSITIVE_Y" &&
    std::isfinite(place_waypoint_clearance_y_margin) && place_waypoint_clearance_y_margin > 0.0 &&
    std::isfinite(place_waypoint_approach_distance) && place_waypoint_approach_distance > 0.0 &&
    place_waypoint_target.size() == 7U && place_waypoint_direction.size() == 3U &&
    std::all_of(
      place_waypoint_target.begin(), place_waypoint_target.end(),
      [](double value) {return std::isfinite(value);}) &&
    std::all_of(place_waypoint_direction.begin(), place_waypoint_direction.end(),
      [](double value) {return std::isfinite(value);}) &&
    std::abs(std::hypot(place_waypoint_direction[0], place_waypoint_direction[1],
      place_waypoint_direction[2]) - 1.0) < 1.0e-6)
  {
    auto & pose = geometry.place_waypoint_target_pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = place_waypoint_target[0];
    pose.pose.position.y = place_waypoint_target[1];
    pose.pose.position.z = place_waypoint_target[2];
    pose.pose.orientation.x = place_waypoint_target[3];
    pose.pose.orientation.y = place_waypoint_target[4];
    pose.pose.orientation.z = place_waypoint_target[5];
    pose.pose.orientation.w = place_waypoint_target[6];
    geometry.place_waypoint_clearance_y_boundary_m = place_waypoint_clearance_boundary;
    geometry.place_waypoint_clearance_y_margin_m = place_waypoint_clearance_y_margin;
    geometry.place_waypoint_approach_distance_m = place_waypoint_approach_distance;
    geometry.place_waypoint_approach_direction_object.x = place_waypoint_direction[0];
    geometry.place_waypoint_approach_direction_object.y = place_waypoint_direction[1];
    geometry.place_waypoint_approach_direction_object.z = place_waypoint_direction[2];
    geometry.place_waypoint_clearance_toward_positive_y = true;
    geometry.place_waypoint_configured = true;
  }
  if (observation_translation.size() == 3U && observation_rotation.size() == 4U &&
    translation.size() == 3U && rotation.size() == 4U && insertion_axis.size() == 3U)
  {
    geometry.observation_to_object.translation.x = observation_translation[0];
    geometry.observation_to_object.translation.y = observation_translation[1];
    geometry.observation_to_object.translation.z = observation_translation[2];
    geometry.observation_to_object.rotation.x = observation_rotation[0];
    geometry.observation_to_object.rotation.y = observation_rotation[1];
    geometry.observation_to_object.rotation.z = observation_rotation[2];
    geometry.observation_to_object.rotation.w = observation_rotation[3];
    geometry.object_to_grasp_tcp.translation.x = translation[0];
    geometry.object_to_grasp_tcp.translation.y = translation[1];
    geometry.object_to_grasp_tcp.translation.z = translation[2];
    geometry.object_to_grasp_tcp.rotation.x = rotation[0];
    geometry.object_to_grasp_tcp.rotation.y = rotation[1];
    geometry.object_to_grasp_tcp.rotation.z = rotation[2];
    geometry.object_to_grasp_tcp.rotation.w = rotation[3];
    geometry.insertion_axis_tcp.x = insertion_axis[0];
    geometry.insertion_axis_tcp.y = insertion_axis[1];
    geometry.insertion_axis_tcp.z = insertion_axis[2];
    geometry.configured = true;
  }
  return geometry;
}

class UnavailableMotionBackend final : public MotionBackend
{
public:
  using StopCallback = std::function<void (uint8_t)>;

  UnavailableMotionBackend(
    bool available, std::chrono::milliseconds availability_delay, bool controllable,
    StopCallback stop_callback)
  : available_(available), availability_delay_(availability_delay),
    controllable_(controllable), stop_callback_(std::move(stop_callback)) {}

  bool available() const override
  {
    std::this_thread::sleep_for(availability_delay_);
    return available_.load();
  }

  bool set_motion_envelope(const MotionEnvelope & envelope) override
  {
    if (!envelope.valid || !std::isfinite(envelope.max_velocity_scale) ||
      !std::isfinite(envelope.max_acceleration_scale) ||
      envelope.max_velocity_scale <= 0.0F || envelope.max_velocity_scale > 1.0F ||
      envelope.max_acceleration_scale <= 0.0F || envelope.max_acceleration_scale > 1.0F)
    {
      return false;
    }
    applied_velocity_scale_.store(envelope.max_velocity_scale);
    applied_acceleration_scale_.store(envelope.max_acceleration_scale);
    applied_envelope_valid_.store(true);
    return true;
  }

  MotionEnvelope applied_motion_envelope() const override
  {
    return MotionEnvelope{
      applied_velocity_scale_.load(), applied_acceleration_scale_.load(),
      applied_envelope_valid_.load()};
  }

  bool submit(const ExecuteTask::Goal &) override
  {
    if (!controllable_ || !available()) {
      return false;
    }
    execution_active_.store(true);
    inactivity_confirmed_.store(false);
    return true;
  }

  void cancel() override
  {
    if (stop_callback_) {
      stop_callback_(arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE);
    }
  }

  void hold() override
  {
    if (stop_callback_) {
      stop_callback_(arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED);
    }
  }

  void stop() override
  {
    if (stop_callback_) {
      stop_callback_(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
    }
  }

  bool execution_active() const override {return execution_active_.load();}
  bool backend_inactivity_confirmed() const override {return inactivity_confirmed_.load();}

  void set_execution_active(bool value) {execution_active_.store(value);}
  void set_inactivity_confirmed(bool value) {inactivity_confirmed_.store(value);}

private:
  std::atomic_bool available_;
  std::chrono::milliseconds availability_delay_;
  bool controllable_;
  StopCallback stop_callback_;
  std::atomic_bool execution_active_{false};
  std::atomic_bool inactivity_confirmed_{true};
  std::atomic<float> applied_velocity_scale_{1.0F};
  std::atomic<float> applied_acceleration_scale_{1.0F};
  std::atomic_bool applied_envelope_valid_{true};
};
}  // namespace

MotionNode::MotionNode(const rclcpp::NodeOptions & options)
: MotionNode(nullptr, options) {}

MotionNode::MotionNode(
  std::shared_ptr<MotionBackend> backend, const rclcpp::NodeOptions & options)
: Node("motion_node", options)
{
  const auto backend_mode = declare_parameter<std::string>("backend_mode", "unavailable");
  const auto robot_description = declare_parameter<std::string>("robot_description", "");
  const auto robot_description_semantic = declare_parameter<std::string>(
    "robot_description_semantic", "");
  const auto kinematics_solver = declare_parameter<std::string>(
    "robot_description_kinematics.arm.kinematics_solver", "");
  const auto kinematics_search_resolution = declare_parameter<double>(
    "robot_description_kinematics.arm.kinematics_solver_search_resolution", 0.0);
  const auto kinematics_timeout = declare_parameter<double>(
    "robot_description_kinematics.arm.kinematics_solver_timeout", 0.0);
  const auto pick_ik_solver = declare_parameter<std::string>(
    "pick_ik_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  const auto pick_ik_solve_type = declare_parameter<std::string>("pick_ik_solve_type", "");
  const auto pick_grasp_ik_timeout_s = declare_parameter<double>("pick_grasp_ik_timeout_s", 1.0);
  const auto pick_approach_ik_timeout_s = declare_parameter<double>(
    "pick_approach_ik_timeout_s", 1.0);
  const auto pick_search_budget_s = declare_parameter<double>("pick_search_budget_s", 100.0);
  const auto pick_ik_timeout_s = declare_parameter<double>("pick_ik_timeout_s", 1.0);
  if (!std::isfinite(pick_grasp_ik_timeout_s) || pick_grasp_ik_timeout_s <= 0.0 ||
    pick_grasp_ik_timeout_s > 30.0 || !std::isfinite(pick_approach_ik_timeout_s) ||
    pick_approach_ik_timeout_s <= 0.0 || pick_approach_ik_timeout_s > 30.0 ||
    !std::isfinite(pick_search_budget_s) || pick_search_budget_s <= 0.0 ||
    pick_search_budget_s > 600.0 || !std::isfinite(pick_ik_timeout_s) ||
    pick_ik_timeout_s <= 0.0 || pick_ik_timeout_s > 30.0)
  {
    throw std::invalid_argument(
            "PICK IK timeouts must be in (0, 30] s and search budget in (0, 600] s");
  }
  const auto pick_max_withdrawal_tilt_deg = declare_parameter<double>(
    "pick_max_withdrawal_tilt_deg", 90.0);
  const auto grasp_occupancy_mode = declare_parameter<std::string>(
    "grasp_occupancy_mode", "FULL_COLLISION");
  (void)parse_grasp_occupancy_mode(grasp_occupancy_mode);
  FingerClearanceParameters finger_clearance;
  finger_clearance.radius_m = declare_parameter<double>("finger_clearance_radius_m", 0.010);
  finger_clearance.contact_span_m = declare_parameter<double>("finger_contact_span_m", 0.020);
  finger_clearance.depth_m = declare_parameter<double>("finger_clearance_depth_m", 0.025);
  validate_finger_clearance_parameters(finger_clearance);
  const auto pick_grasp_pose_order_policy = declare_parameter<std::string>(
    "pick_grasp_pose_order_policy", "distance_then_tilt");
  (void)parse_pick_grasp_pose_order_policy(pick_grasp_pose_order_policy);
  const auto pick_position_range_half_length_fraction = declare_parameter<double>(
    "pick_position_range_half_length_fraction", 0.5);
  const auto pick_max_downward_position_offset_m = declare_parameter<double>(
    "pick_max_downward_position_offset_m", 0.005);
  const auto pick_position_offset_samples_per_tangent_axis = declare_parameter<int64_t>(
    "pick_position_offset_samples_per_tangent_axis", 3);
  const auto pick_orientation_samples_per_closing_axis = declare_parameter<int64_t>(
    "pick_orientation_samples_per_closing_axis", 8);
  if (!std::isfinite(pick_max_withdrawal_tilt_deg) || pick_max_withdrawal_tilt_deg < 0.0 ||
    pick_max_withdrawal_tilt_deg > 180.0 ||
    !std::isfinite(pick_position_range_half_length_fraction) ||
    pick_position_range_half_length_fraction < 0.0 ||
    pick_position_range_half_length_fraction >= 1.0 ||
    !std::isfinite(pick_max_downward_position_offset_m) ||
    pick_max_downward_position_offset_m < 0.0 ||
    pick_position_offset_samples_per_tangent_axis < 1 ||
    pick_position_offset_samples_per_tangent_axis > 20 ||
    pick_orientation_samples_per_closing_axis < 1 || pick_orientation_samples_per_closing_axis > 72)
  {
    throw std::invalid_argument("invalid PICK grasp search sampling configuration");
  }
  const auto pick_grasp_pose_candidate_limit = declare_parameter<int64_t>(
    "pick_grasp_pose_candidate_limit", 48);
  if (pick_grasp_pose_candidate_limit < 3 || pick_grasp_pose_candidate_limit > 1000) {
    throw std::invalid_argument("pick_grasp_pose_candidate_limit must be in [3, 1000]");
  }
  const auto pick_ik_seed_attempts_per_grasp_pose = declare_parameter<int64_t>(
    "pick_ik_seed_attempts_per_grasp_pose", 2);
  if (pick_ik_seed_attempts_per_grasp_pose < 1) {
    throw std::invalid_argument("pick_ik_seed_attempts_per_grasp_pose must be at least 1");
  }
  const auto positive_profile_limit = [&](const char * name, int64_t fallback,
      int64_t maximum) {
      const auto value = declare_parameter<int64_t>(name, fallback);
      if (value < 1 || value > maximum) {
        throw std::invalid_argument(std::string(name) + " is outside its supported range");
      }
      return value;
    };
  const auto place_max_pose_candidates =
    positive_profile_limit("place_max_pose_candidates", 13, 100);
  const auto pick_approach_global_max_requests = positive_profile_limit(
    "pick_approach_global_max_requests", 3, 20);
  const auto pick_final_cartesian_max_requests = positive_profile_limit(
    "pick_final_cartesian_max_requests", 1, 20);
  const auto pick_final_global_fallback_max_requests = positive_profile_limit(
    "pick_final_global_fallback_max_requests", 3, 20);
  const auto place_approach_global_max_requests = positive_profile_limit(
    "place_approach_global_max_requests", 3, 20);
  const auto place_final_cartesian_max_requests = positive_profile_limit(
    "place_final_cartesian_max_requests", 1, 20);
  const auto place_final_global_fallback_max_requests = positive_profile_limit(
    "place_final_global_fallback_max_requests", 3, 20);
  const auto availability_delay_ms = declare_parameter<int64_t>(
    "backend_availability_delay_ms", 0);
  const auto backend_available = declare_parameter<bool>("backend_available", false);
  const auto enable_test_backend_controls = declare_parameter<bool>(
    "enable_test_backend_controls", false);
  const auto holding_confirmation_timeout_ms = declare_parameter<int64_t>(
    "holding_confirmation_timeout_ms", 500);
  const auto planning_velocity_scaling_factor = declare_parameter<double>(
    "planning_velocity_scaling_factor", 0.1);
  const auto planning_acceleration_scaling_factor = declare_parameter<double>(
    "planning_acceleration_scaling_factor", 0.1);
  const auto disturbance_push_distance_m = declare_parameter<double>(
    "disturbance_push_distance_m", 0.02);
  const auto disturbance_approach_clearance_m = declare_parameter<double>(
    "disturbance_approach_clearance_m", 0.01);
  const auto disturbance_speed_fraction = declare_parameter<double>(
    "disturbance_speed_fraction", 0.2);
  const auto planning_time_s = declare_parameter<double>("planning_time_s", 5.0);
  const auto planning_num_attempts = declare_parameter<int64_t>("planning_num_attempts", 3);
  if (planning_num_attempts < 1) {
    throw std::invalid_argument("planning_num_attempts must be at least 1");
  }
  const auto planning_planner_id = declare_parameter<std::string>(
    "planning_planner_id", "RRTConnect");
  const auto place_tray_return_joint_positions = declare_parameter<std::vector<double>>(
    "place_tray_return_joint_positions", std::vector<double>{});
  const auto place_candidate_planning_time_s = declare_parameter<double>(
    "place_candidate_planning_time_s", 1.0);
  const auto place_total_planning_time_s = declare_parameter<double>(
    "place_total_planning_time_s", 5.0);
  const auto cartesian_tcp_orientation_tolerance_rad = declare_parameter<double>(
    "cartesian_tcp_orientation_tolerance_rad", 0.08726646259971647);
  const auto motion_progress_stall_timeout_s = declare_parameter<double>(
    "motion_progress_stall_timeout_s", 60.0);
  const auto trajectory_goal_tolerance_rad = declare_parameter<double>(
    "trajectory_goal_tolerance_rad", 0.03);
  const auto diagnostics_capture_enabled = declare_parameter<bool>(
    "diagnostics.capture_enabled", false,
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Enable bounded optional Motion clock diagnostics independently of log level."));
  const auto diagnostics_output_directory = declare_parameter<std::string>(
    "diagnostics.output_directory", ".local_artifacts/observability");
  const auto diagnostics_snapshot_run_directory = declare_parameter<std::string>(
    "diagnostics.snapshot_run_directory", ".local_artifacts/snapshots");
  const auto place_waypoint_trajectory_file = declare_parameter<std::string>(
    "place_waypoint_trajectory_file", "");
  if (holding_confirmation_timeout_ms <= 0) {
    throw std::invalid_argument("holding_confirmation_timeout_ms must be positive");
  }
  const auto motion_geometry = read_motion_geometry(*this);
  runtime_geometry_ = motion_geometry;
  holding_confirmation_timeout_ = std::chrono::milliseconds(holding_confirmation_timeout_ms);
  runtime_tuning_.planning_time_s = planning_time_s;
  runtime_tuning_.place_candidate_planning_time_s = place_candidate_planning_time_s;
  runtime_tuning_.place_total_planning_time_s = place_total_planning_time_s;
  runtime_tuning_.planning_velocity_scaling_factor = planning_velocity_scaling_factor;
  runtime_tuning_.planning_acceleration_scaling_factor = planning_acceleration_scaling_factor;
  runtime_tuning_.disturbance_push_distance_m = disturbance_push_distance_m;
  runtime_tuning_.disturbance_approach_clearance_m = disturbance_approach_clearance_m;
  runtime_tuning_.disturbance_speed_fraction = disturbance_speed_fraction;
  runtime_tuning_.motion_progress_stall_timeout_s = motion_progress_stall_timeout_s;
  runtime_tuning_.cartesian_tcp_orientation_tolerance_rad =
    cartesian_tcp_orientation_tolerance_rad;
  runtime_tuning_.trajectory_goal_tolerance_rad = trajectory_goal_tolerance_rad;

  rclcpp::Publisher<arm_cell_interfaces::msg::StopMode>::SharedPtr stop_mode_publisher;
  if (enable_test_backend_controls) {
    stop_mode_publisher = create_publisher<arm_cell_interfaces::msg::StopMode>(
      "/motion/test_backend/stop_mode", rclcpp::QoS(10));
  }
  std::shared_ptr<UnavailableMotionBackend> test_backend;
  if (!backend) {
    if (backend_mode == "isaac") {
      if (enable_test_backend_controls) {
        throw std::invalid_argument("test backend controls are not valid for the Isaac backend");
      }
      const auto arm_joint_names = declare_parameter<std::vector<std::string>>(
        "arm_joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
      const auto home_joint_positions = declare_parameter<std::vector<double>>(
        "home_joint_positions", std::vector<double>{});
      const auto retract_joint_positions = declare_parameter<std::vector<double>>(
        "retract_joint_positions", std::vector<double>{});
      if (place_tray_return_joint_positions.size() != arm_joint_names.size() ||
        !std::all_of(
          place_tray_return_joint_positions.begin(),
          place_tray_return_joint_positions.end(), [](double value) {return std::isfinite(value);}))
      {
        throw std::invalid_argument(
          "place_tray_return_joint_positions must contain one finite value per arm_joint_name");
      }
      const auto validation_grasp_contact_policy_enabled = declare_parameter<bool>(
        "validation_grasp_contact_policy_enabled", false);
      const auto validation_grasp_contact_object_id_prefix = declare_parameter<std::string>(
        "validation_grasp_contact_object_id_prefix", "");
      const auto validation_grasp_contact_links = declare_parameter<std::vector<std::string>>(
        "validation_grasp_contact_links", std::vector<std::string>{});
      const auto planning_max_acceleration_rad_s2 = declare_parameter<double>(
        "planning_max_acceleration_rad_s2", 0.0);
      if (!std::isfinite(place_candidate_planning_time_s) ||
        place_candidate_planning_time_s <= 0.0 ||
        !std::isfinite(place_total_planning_time_s) || place_total_planning_time_s <= 0.0)
      {
        throw std::invalid_argument("PLACE planning budgets must be finite and positive");
      }
      const auto trajectory_sampler = declare_parameter<std::string>(
        "trajectory_sampler", "linear");
      const auto trajectory_clock_liveness_timeout_s = declare_parameter<double>(
        "trajectory_clock_liveness_timeout_s", 2.0);

      rclcpp::NodeOptions backend_options;
      std::vector<rclcpp::Parameter> backend_parameters;
      if (has_parameter("use_sim_time")) {
        backend_parameters.emplace_back(
          "use_sim_time", get_parameter("use_sim_time").as_bool());
      }
      if (!robot_description.empty()) {
        backend_parameters.emplace_back("robot_description", robot_description);
      }
      if (!robot_description_semantic.empty()) {
        backend_parameters.emplace_back("robot_description_semantic", robot_description_semantic);
      }
      if (!kinematics_solver.empty()) {
        backend_parameters.emplace_back(
          "robot_description_kinematics.arm.kinematics_solver", kinematics_solver);
        backend_parameters.emplace_back(
          "robot_description_kinematics.arm.kinematics_solver_search_resolution",
          kinematics_search_resolution);
        backend_parameters.emplace_back(
          "robot_description_kinematics.arm.kinematics_solver_timeout", kinematics_timeout);
      }
      backend_parameters.emplace_back("pick_ik_solver", pick_ik_solver);
      backend_parameters.emplace_back("pick_ik_solve_type", pick_ik_solve_type);
      backend_parameters.emplace_back("pick_grasp_ik_timeout_s", pick_grasp_ik_timeout_s);
      backend_parameters.emplace_back("grasp_occupancy_mode", grasp_occupancy_mode);
      backend_parameters.emplace_back("finger_clearance_radius_m", finger_clearance.radius_m);
      backend_parameters.emplace_back("finger_contact_span_m", finger_clearance.contact_span_m);
      backend_parameters.emplace_back("finger_clearance_depth_m", finger_clearance.depth_m);
      backend_parameters.emplace_back("pick_approach_ik_timeout_s", pick_approach_ik_timeout_s);
      backend_parameters.emplace_back("pick_search_budget_s", pick_search_budget_s);
      backend_parameters.emplace_back("pick_ik_timeout_s", pick_ik_timeout_s);
      backend_parameters.emplace_back(
        "pick_max_withdrawal_tilt_deg", pick_max_withdrawal_tilt_deg);
      backend_parameters.emplace_back(
        "pick_grasp_pose_order_policy", pick_grasp_pose_order_policy);
      backend_parameters.emplace_back(
        "pick_position_range_half_length_fraction",
        pick_position_range_half_length_fraction);
      backend_parameters.emplace_back(
        "pick_max_downward_position_offset_m", pick_max_downward_position_offset_m);
      backend_parameters.emplace_back(
        "pick_position_offset_samples_per_tangent_axis",
        pick_position_offset_samples_per_tangent_axis);
      backend_parameters.emplace_back(
        "pick_orientation_samples_per_closing_axis",
        pick_orientation_samples_per_closing_axis);
      backend_parameters.emplace_back(
        "pick_grasp_pose_candidate_limit", pick_grasp_pose_candidate_limit);
      backend_parameters.emplace_back(
        "pick_ik_seed_attempts_per_grasp_pose", pick_ik_seed_attempts_per_grasp_pose);
      backend_parameters.emplace_back("place_max_pose_candidates", place_max_pose_candidates);
      backend_parameters.emplace_back(
        "place_tray_return_joint_positions", place_tray_return_joint_positions);
      backend_parameters.emplace_back(
        "pick_approach_global_max_requests", pick_approach_global_max_requests);
      backend_parameters.emplace_back(
        "pick_final_cartesian_max_requests", pick_final_cartesian_max_requests);
      backend_parameters.emplace_back(
        "pick_final_global_fallback_max_requests", pick_final_global_fallback_max_requests);
      backend_parameters.emplace_back(
        "place_approach_global_max_requests", place_approach_global_max_requests);
      backend_parameters.emplace_back(
        "place_final_cartesian_max_requests", place_final_cartesian_max_requests);
      backend_parameters.emplace_back(
        "place_final_global_fallback_max_requests", place_final_global_fallback_max_requests);
      backend_parameters.emplace_back("planning_num_attempts", planning_num_attempts);
      if (!backend_parameters.empty()) {
        backend_options.parameter_overrides(backend_parameters);
      }
      backend_options.use_global_arguments(false);
      backend_node_ = std::make_shared<rclcpp::Node>("isaac_motion_backend", backend_options);
      isaac_transport_ = std::make_shared<IsaacRosMotionTransport>(
        backend_node_, arm_joint_names, home_joint_positions, retract_joint_positions,
        false, validation_grasp_contact_policy_enabled,
        validation_grasp_contact_object_id_prefix, validation_grasp_contact_links,
        planning_max_acceleration_rad_s2, trajectory_sampler,
        trajectory_clock_liveness_timeout_s,
        cartesian_tcp_orientation_tolerance_rad,
        motion_progress_stall_timeout_s, trajectory_goal_tolerance_rad,
        planning_velocity_scaling_factor,
        planning_acceleration_scaling_factor, planning_time_s,
        place_candidate_planning_time_s, place_total_planning_time_s,
        diagnostics_capture_enabled, diagnostics_output_directory,
        diagnostics_snapshot_run_directory, planning_planner_id,
        place_waypoint_trajectory_file);
      backend = std::make_shared<IsaacMotionBackend>(isaac_transport_);
    } else if (backend_mode == "unavailable") {
      test_backend = std::make_shared<UnavailableMotionBackend>(
        backend_available, std::chrono::milliseconds(availability_delay_ms),
        enable_test_backend_controls,
        [stop_mode_publisher](uint8_t mode) {
          if (stop_mode_publisher) {
            arm_cell_interfaces::msg::StopMode message;
            message.value = mode;
            stop_mode_publisher->publish(message);
          }
        });
      backend = test_backend;
    } else {
      throw std::invalid_argument("unsupported Motion backend_mode: " + backend_mode);
    }
  }

  core_ = std::make_shared<MotionCore>(
    backend, std::chrono::milliseconds(500), motion_geometry,
    std::chrono::milliseconds(holding_confirmation_timeout_ms));
  if (!core_->set_runtime_tuning(
      runtime_tuning_, runtime_geometry_, holding_confirmation_timeout_))
  {
    throw std::runtime_error("failed to initialize Motion tuning snapshot");
  }
  tuning_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      std::lock_guard<std::mutex> parameter_lock(tuning_parameters_mutex_);
      rcl_interfaces::msg::SetParametersResult result;
      auto tuning = runtime_tuning_;
      auto geometry = runtime_geometry_;
      auto holding_timeout = holding_confirmation_timeout_;
      bool idle_mutable_update = false;
      bool strategy_declaration = false;
      auto set_double = [&](const rclcpp::Parameter & parameter, double & destination) {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.reason = parameter.get_name() + " must be a double";
            return false;
          }
          destination = parameter.as_double();
          return true;
        };
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == "pick_strategy" || name == "place_strategy") {
          // Task Execution declares these read-only startup parameters after
          // MotionNode construction. Accept their initial declaration here;
          // the descriptor rejects subsequent runtime updates.
          strategy_declaration = true;
          continue;
        }
        double * target = nullptr;
        if (name == "planning_time_s") {
          target = &tuning.planning_time_s;
        } else if (name == "place_candidate_planning_time_s") {
          target = &tuning.place_candidate_planning_time_s;
        } else if (name == "place_total_planning_time_s") {
          target = &tuning.place_total_planning_time_s;
        } else if (name == "planning_velocity_scaling_factor") {
          target = &tuning.planning_velocity_scaling_factor;
        } else if (name == "planning_acceleration_scaling_factor") {
          target = &tuning.planning_acceleration_scaling_factor;
        } else if (name == "disturbance_push_distance_m") {
          target = &tuning.disturbance_push_distance_m;
        } else if (name == "disturbance_approach_clearance_m") {
          target = &tuning.disturbance_approach_clearance_m;
        } else if (name == "disturbance_speed_fraction") {
          target = &tuning.disturbance_speed_fraction;
        } else if (name == "motion_progress_stall_timeout_s") {
          target = &tuning.motion_progress_stall_timeout_s;
          idle_mutable_update = true;
        } else if (name == "cartesian_tcp_orientation_tolerance_rad") {
          target = &tuning.cartesian_tcp_orientation_tolerance_rad;
          idle_mutable_update = true;
        } else if (name == "trajectory_goal_tolerance_rad") {
          target = &tuning.trajectory_goal_tolerance_rad;
          idle_mutable_update = true;
        } else if (name == "holding_confirmation_timeout_ms") {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            result.reason = name + " must be an integer";
            return result;
          }
          const auto value = parameter.as_int();
          if (value < 1 || value > 60000) {
            result.reason = name + " must be in [1, 60000] ms";
            return result;
          }
          holding_timeout = std::chrono::milliseconds(value);
          idle_mutable_update = true;
          continue;
        } else {
          result.reason = name + " is static/restart-required or not runtime tuning";
          return result;
        }
        if (!set_double(parameter, *target)) {
          return result;
        }
      }
      if (strategy_declaration) {
        result.successful = parameters.size() == 1;
        result.reason = result.successful ? "startup strategy declaration accepted" :
          "strategy declaration cannot be combined with runtime tuning";
        return result;
      }
      const auto in_range = [](double value, double minimum, double maximum) {
          return std::isfinite(value) && value >= minimum && value <= maximum;
        };
      if (!in_range(tuning.planning_time_s, 0.05, 300.0) ||
        !in_range(tuning.place_candidate_planning_time_s, 0.05, 60.0) ||
        !in_range(tuning.place_total_planning_time_s, 0.05, 300.0) ||
        tuning.place_total_planning_time_s < tuning.place_candidate_planning_time_s ||
        !in_range(tuning.planning_velocity_scaling_factor, 1.0e-4, 1.0) ||
        !in_range(tuning.planning_acceleration_scaling_factor, 1.0e-4, 1.0) ||
        !in_range(tuning.disturbance_push_distance_m, 0.001, 0.1) ||
        !in_range(tuning.disturbance_approach_clearance_m, 0.001, 0.1) ||
        !in_range(tuning.disturbance_speed_fraction, 1.0e-3, 1.0) ||
        !in_range(tuning.motion_progress_stall_timeout_s, 0.1, 600.0) ||
        !in_range(tuning.cartesian_tcp_orientation_tolerance_rad, 1.0e-5, 0.5) ||
        !in_range(tuning.trajectory_goal_tolerance_rad, 1.0e-5, 1.0) ||
        !in_range(geometry.pick_retract_distance_m, 1.0e-4, 0.5))
      {
        result.reason = "Motion tuning value is outside its finite supported range";
        return result;
      }
      if (!core_->set_runtime_tuning(
          tuning, geometry, holding_timeout, idle_mutable_update))
      {
        result.reason = "idle-mutable Motion tuning requires Motion to be idle";
        return result;
      }
      runtime_tuning_ = tuning;
      runtime_geometry_ = geometry;
      holding_confirmation_timeout_ = holding_timeout;
      result.successful = true;
      result.reason = "accepted for the next Motion action; the active action keeps its snapshot";
      return result;
    });
  safety_state_subscription_ = create_subscription<SafetyState>(
    "/safety/state", rclcpp::QoS(10),
    [this](const SafetyState::SharedPtr message) {
      core_->update_safety_state(*message, MotionCore::Clock::now());
    });
  status_publisher_ = create_publisher<MotionStatus>("/motion/state", rclcpp::QoS(10));
  stop_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  stop_service_ = create_service<arm_cell_interfaces::srv::StopMotion>(
    "/safety/stop_motion",
    [this](
      const std::shared_ptr<arm_cell_interfaces::srv::StopMotion::Request> request,
      std::shared_ptr<arm_cell_interfaces::srv::StopMotion::Response> response) {
      response->accepted = core_->request_stop(request->request_id, request->stop_mode.value);
      response->diagnostic_detail = response->accepted ?
      "Stop request accepted for processing" : "No active Motion execution or invalid stop mode";
    }, rmw_qos_profile_services_default, stop_callback_group_);
  gripper_open_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  gripper_open_service_ = create_service<arm_cell_interfaces::srv::RequestGripperOpen>(
    "/motion/operator/gripper_open",
    [this](
      const std::shared_ptr<arm_cell_interfaces::srv::RequestGripperOpen::Request> request,
      std::shared_ptr<arm_cell_interfaces::srv::RequestGripperOpen::Response> response) {
      const auto result = core_->request_operator_gripper_open(
        request->object_supported_or_restrained, request->release_area_clear);
      response->accepted = result.accepted;
      response->command_issued = result.command_issued;
      response->released_confirmed = result.released_confirmed;
      response->diagnostic_detail = result.diagnostic_detail;
    }, rmw_qos_profile_services_default, gripper_open_callback_group_);
  place_waypoint_generate_service_ = create_service<std_srvs::srv::Trigger>(
    "/motion/generate_place_waypoint_trajectory",
    [this, place_waypoint_trajectory_file](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      std::string detail;
      response->success = core_->generatePlaceWaypointApproach(
        place_waypoint_trajectory_file, detail);
      response->message = detail;
    });
  if (test_backend) {
    test_backend_active_service_ = create_service<std_srvs::srv::SetBool>(
      "/motion/test_backend/execution_active",
      [test_backend](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        test_backend->set_execution_active(request->data);
        response->success = true;
        response->message = "fake backend execution_active updated";
      });
    test_backend_inactivity_service_ = create_service<std_srvs::srv::SetBool>(
      "/motion/test_backend/inactivity_confirmed",
      [test_backend](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        test_backend->set_inactivity_confirmed(request->data);
        response->success = true;
        response->message = "fake backend inactivity confirmation updated";
      });
  }
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() {
      core_->refresh_stop_state();
      auto status = core_->status();
      status.header.stamp = now();
      status_publisher_->publish(status);
    });
}

void MotionNode::initialize_backend()
{
  if (isaac_transport_) {
    isaac_transport_->initialize_move_group();
  }
}

void MotionNode::configure_waypoint_mode(bool pick_waypoint, bool place_waypoint)
{
  if (isaac_transport_) {
    isaac_transport_->configure_waypoint_mode(pick_waypoint, place_waypoint);
  }
}

}  // namespace arm_cell_motion
