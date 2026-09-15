# Robotiq Description Vendor

Upstream:
robotiq/ros

Imported revision:
<commit hash>

Purpose:
Vendored and adapted for SF-Twin standalone Isaac Sim integration.

Local modifications:
- Removed dependency on ROS package lookup where required.
- Adapted Xacro include paths for project-local builds.
- Adapted mesh URI resolution for standalone Isaac Sim import.
- Original robot geometry, kinematics, inertial parameters, and joint
  definitions are preserved unless explicitly documented otherwise.

Original license:
See LICENSE.
