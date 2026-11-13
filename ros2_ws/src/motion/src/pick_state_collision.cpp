#include "arm_cell_motion/pick_state_collision.hpp"

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <fcl/narrowphase/collision.h>
#include <algorithm>

namespace arm_cell_motion
{
PickCollisionValidationResult validate_pick_state_collision(
  const planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state, const std::string & planning_group)
{
  collision_detection::CollisionRequest request;
  request.group_name = planning_group;
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
  const geometry_msgs::msg::PoseStamped & tcp_goal,
  const std::vector<std::string> & clearance_objects)
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
      if (std::find(clearance_objects.begin(), clearance_objects.end(), id) ==
        clearance_objects.end()) {continue;}
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

void append_pick_clearance_scene_occupancy(
  const planning_scene::PlanningScene & scene,
  const std::vector<std::string> & clearance_objects,
  std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & occupancy)
{
  const auto & world = scene.getWorld();
  for (const auto & id : clearance_objects) {
    const auto rail = world->getObject(id);
    if (!rail) {continue;}
    for (std::size_t i = 0; i < rail->shapes_.size(); ++i) {
      if (rail->shapes_[i]->type != shapes::BOX) {continue;}
      const auto & box = static_cast<const shapes::Box &>(*rail->shapes_[i]);
      const auto & transform = rail->global_shape_poses_[i];
      arm_cell_interfaces::msg::CollisionPrimitive primitive;
      primitive.shape_type =
        arm_cell_interfaces::msg::CollisionPrimitive::SOLID_PRIMITIVE;
      primitive.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.shape.dimensions.assign(box.size, box.size + 3);
      primitive.pose.position.x = transform.translation().x();
      primitive.pose.position.y = transform.translation().y();
      primitive.pose.position.z = transform.translation().z();
      const Eigen::Quaterniond orientation(transform.linear());
      primitive.pose.orientation.x = orientation.x();
      primitive.pose.orientation.y = orientation.y();
      primitive.pose.orientation.z = orientation.z();
      primitive.pose.orientation.w = orientation.w();
      occupancy.push_back(std::move(primitive));
    }
  }
}
}  // namespace arm_cell_motion
