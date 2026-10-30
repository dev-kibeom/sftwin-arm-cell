#pragma once

#include <chrono>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>

#include "arm_cell_motion/cartesian_manipulation.hpp"

namespace arm_cell_motion
{

class CartesianPlanningTimer final
{
public:
  CartesianPlanningTimer()
  : start_(std::chrono::steady_clock::now())
  {
  }

  double elapsed_s() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_).count();
  }

private:
  std::chrono::steady_clock::time_point start_;
};

inline void append_cartesian_planning_elapsed(
  std::ostream & output, std::optional<double> elapsed_s)
{
  output << ",\"planning_elapsed_s\":";
  if (elapsed_s) {
    output << *elapsed_s << ",\"planning_elapsed_status\":\"MEASURED\"";
  } else {
    output << "null,\"planning_elapsed_status\":\"NOT_STARTED\"";
  }
}

inline void append_cartesian_planning_elapsed(
  nlohmann::json & output, std::optional<double> elapsed_s)
{
  output["planning_elapsed_s"] = elapsed_s ? nlohmann::json(*elapsed_s) : nlohmann::json(nullptr);
  output["planning_elapsed_status"] = elapsed_s ? "MEASURED" : "NOT_STARTED";
}

inline nlohmann::json cartesian_pose_json(const geometry_msgs::msg::PoseStamped & pose)
{
  return {
    {"frame_id", pose.header.frame_id},
    {"position_xyz", {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z}},
    {"orientation_xyzw", {pose.pose.orientation.x, pose.pose.orientation.y,
      pose.pose.orientation.z, pose.pose.orientation.w}}
  };
}

inline std::string make_pick_cartesian_diagnostic(
  const std::string & motion_goal_uuid,
  const std::string & stage,
  const std::string & status,
  const std::string & failure_reason,
  const std::string & direction,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & target,
  const geometry_msgs::msg::PoseStamped & requested_pose,
  const CartesianLinearGenerationResult & result,
  std::optional<double> planning_elapsed_s)
{
  const bool failure_sample_available = !result.failure.reason.empty();
  nlohmann::json contact_bodies = {
    {"available", false},
    {"unavailable_reason", !failure_sample_available ? "no_failed_path_sample" :
      result.failure.reason == "collision/contact" ?
      "state_validity_returned_no_contact_pairs" :
      result.failure.reason == "state validity check failed" ?
      "state_validity_response_without_contact_pairs" : "contact_check_not_reached"},
    {"pairs", nlohmann::json::array()}
  };
  if (failure_sample_available && !result.failure.contact_bodies.empty()) {
    auto & pairs = contact_bodies["pairs"];
    for (std::size_t index = 0; index + 1U < result.failure.contact_bodies.size(); index += 2U) {
      pairs.push_back({result.failure.contact_bodies[index], result.failure.contact_bodies[index + 1U]});
    }
    contact_bodies["available"] = result.failure.contact_bodies.size() % 2U == 0U;
    contact_bodies["unavailable_reason"] = contact_bodies["available"].get<bool>() ?
      "" : "incomplete_contact_body_pair";
  }

  nlohmann::json failure_sample = {
    {"available", failure_sample_available},
    {"unavailable_reason", failure_sample_available ? "" : "no_failed_path_sample"},
    {"pose_available", failure_sample_available && result.failure.tcp_available},
    {"contact_bodies", std::move(contact_bodies)}
  };
  if (failure_sample_available) {
    failure_sample["index"] = result.failure.sample_index;
    failure_sample["validation_reason"] = result.failure.reason;
    if (result.failure.tcp_available) {
      auto sample_pose = start;
      sample_pose.pose = result.failure.tcp;
      failure_sample["pose"] = cartesian_pose_json(sample_pose);
      failure_sample["pose_unavailable_reason"] = "";
    } else {
      failure_sample["pose_unavailable_reason"] = "forward_kinematics_not_captured";
    }
  }

  nlohmann::json diagnostic = {
    {"motion_goal_uuid", motion_goal_uuid},
    {"phase", stage == "cartesian_retract" ? "RETRACTING" : "EXECUTING"},
    {"stage", stage},
    {"status", status},
    {"direction", direction},
    {"start", cartesian_pose_json(start)},
    {"target", cartesian_pose_json(target)},
    {stage == "cartesian_retract" ? "requested_retract_pose" : "requested_grasp_pose",
      cartesian_pose_json(requested_pose)},
    {"path_fraction", result.path_fraction},
    {"sample_count", result.sample_count},
    {"validated_sample_count", result.validated_sample_count},
    {"failure_reason", failure_reason.empty() ? result.reason : failure_reason},
    {"failure_sample", std::move(failure_sample)}
  };
  append_cartesian_planning_elapsed(diagnostic, planning_elapsed_s);
  return diagnostic.dump();
}

inline std::string upsert_cartesian_diagnostic(
  std::string evidence, const std::string & key, const std::string & value)
{
  auto object = evidence.empty() ? nlohmann::json::object() :
    nlohmann::json::parse(evidence, nullptr, false);
  auto member = nlohmann::json::parse(value, nullptr, false);
  if (object.is_discarded() || !object.is_object() || member.is_discarded()) {
    return evidence;
  }
  object[key] = std::move(member);
  return object.dump();
}

inline bool cartesian_result_selected_for_execution(const std::string & strategy)
{
  return strategy == "CARTESIAN";
}

inline std::string upsert_cartesian_ready_diagnostic(
  std::string evidence, const std::string & key, const std::string & value,
  const std::string & selected_strategy)
{
  if (!cartesian_result_selected_for_execution(selected_strategy)) {
    return evidence;
  }
  auto diagnostic = nlohmann::json::parse(value, nullptr, false);
  if (!diagnostic.is_object()) {
    return evidence;
  }
  diagnostic["selected_strategy"] = selected_strategy;
  return upsert_cartesian_diagnostic(std::move(evidence), key, diagnostic.dump());
}

}  // namespace arm_cell_motion
