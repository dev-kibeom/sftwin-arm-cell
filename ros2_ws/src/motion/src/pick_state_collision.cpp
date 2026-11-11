#include "arm_cell_motion/pick_state_collision.hpp"

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <fcl/narrowphase/collision.h>

namespace arm_cell_motion
{
PickCollisionValidationResult validate_pick_state_collision(
  const planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state)
{
  collision_detection::CollisionRequest request;
  request.group_name = "arm";
  request.contacts = true;
  request.max_contacts = 32;
  collision_detection::CollisionResult collision;
  scene.checkCollision(request, collision, state);
  PickCollisionValidationResult result;
  result.collision = collision.collision;
  for (const auto & contact_pair : collision.contacts) {
    result.contacts.emplace_back(contact_pair.first.first, contact_pair.first.second);
  }
  return result;
}
PickCollisionValidationResult validate_pick_grasp_pose_rail_collision(
  const planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * arm,
  const geometry_msgs::msg::PoseStamped & tcp_goal)
{
  PickCollisionValidationResult result;
  const auto model = state.getRobotModel();
  const auto * tcp = model->getLinkModel("sf_grasp_tcp");
  if (!tcp || !arm || tcp_goal.header.frame_id != model->getModelFrame()) {return result;}
  const auto arm_ancestors = [arm](const moveit::core::LinkModel * link) {
      std::vector<std::string> joints;
      while (link) {
        const auto * joint = link->getParentJointModel();
        if (joint && joint->getVariableCount() && arm->hasJointModel(joint->getName())) {
          joints.push_back(joint->getName());
        }
        link = link->getParentLinkModel();
      }
      return joints;
    };
  const auto tcp_ancestors = arm_ancestors(tcp);
  const auto & p = tcp_goal.pose.position;
  const auto & q = tcp_goal.pose.orientation;
  Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-9) {return result;}
  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  goal.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  if (!goal.translation().allFinite()) {return result;}
  goal.linear() = orientation.normalized().toRotationMatrix();
  moveit::core::RobotState reference(state);
  reference.update();
  const Eigen::Isometry3d relocate = goal * reference.getGlobalLinkTransform(tcp).inverse();
  const auto & env = *scene.getCollisionEnv();
  for (const auto * link : model->getLinkModelsWithCollisionGeometry()) {
    const auto & name = link->getName();
    if (name.rfind("gripper_", 0U) != 0U || arm_ancestors(link) != tcp_ancestors) {continue;}
    for (const auto & id : scene.getWorld()->getObjectIds()) {
      if (id.rfind("amr_tray_rail_", 0U) != 0U) {continue;}
      collision_detection::AllowedCollision::Type allowed;
      if (scene.getAllowedCollisionMatrix().getAllowedCollision(name, id, allowed) &&
        allowed != collision_detection::AllowedCollision::NEVER) {continue;}
      const auto object = scene.getWorld()->getObject(id);
      bool colliding = false;
      for (std::size_t i = 0U; i < link->getShapes().size() && !colliding; ++i) {
        const auto geometry = collision_detection::createCollisionGeometry(
          link->getShapes()[i], env.getLinkScale(name), env.getLinkPadding(name), link, i);
        if (!geometry) {continue;}
        const Eigen::Isometry3d transform = relocate * reference.getCollisionBodyTransform(link, i);
        for (std::size_t j = 0U; j < object->shapes_.size(); ++j) {
          const auto rail = collision_detection::createCollisionGeometry(
            object->shapes_[j],
            object.get());
          if (!rail) {continue;}
          fcl::CollisionRequestd request;
          fcl::CollisionResultd collision;
          fcl::collide(
            geometry->collision_geometry_.get(), transform,
            rail->collision_geometry_.get(), object->global_shape_poses_[j], request, collision);
          if (collision.isCollision()) {colliding = true; break;}
        }
      }
      if (colliding) {result.collision = true; result.contacts.emplace_back(id, name);}
    }
  }
  return result;
}
}  // namespace arm_cell_motion
