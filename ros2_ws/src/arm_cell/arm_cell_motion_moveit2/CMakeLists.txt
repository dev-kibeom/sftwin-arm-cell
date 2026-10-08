cmake_minimum_required(VERSION 3.8)
project(arm_cell_motion_moveit2)

execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../.."
  OUTPUT_VARIABLE ARM_CELL_BUILD_COMMIT_SHA
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
execute_process(
  COMMAND git status --porcelain
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../.."
  OUTPUT_VARIABLE ARM_CELL_SOURCE_STATE
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(NOT ARM_CELL_BUILD_COMMIT_SHA)
  set(ARM_CELL_BUILD_COMMIT_SHA "commit-sha-unreported")
endif()
if(ARM_CELL_SOURCE_STATE)
  string(APPEND ARM_CELL_BUILD_COMMIT_SHA "-dirty")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(arm_cell_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/motion_core.cpp
  src/isaac_motion_backend.cpp
  src/isaac_gripper_port.cpp
  src/isaac_ros_motion_transport.cpp
  src/trajectory_validation.cpp
  src/executable_trajectory_finalize.cpp
  src/trajectory_sampler.cpp
  src/trajectory_execution_worker.cpp
  src/motion_profile_trace.cpp
  src/trajectory_settling_diagnostics.cpp
  src/trajectory_watchdog.cpp
  src/motion_progress_watchdog.cpp
  src/final_target_hold.cpp
  src/trajectory_continuity.cpp
  src/place_approach_ik_policy.cpp
  src/place_approach_trajectory.cpp
  src/place_object_pose_candidates.cpp
  src/cartesian_manipulation.cpp
  src/approach_entry_acceptance.cpp
  src/object_state_tracker.cpp
  src/task_executor.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME}_core PRIVATE
  ARM_CELL_BUILD_COMMIT_SHA="${ARM_CELL_BUILD_COMMIT_SHA}"
)
ament_target_dependencies(${PROJECT_NAME}_core
  arm_cell_interfaces
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  rosgraph_msgs
  tf2
)

add_library(${PROJECT_NAME}_server
  src/motion_action_server.cpp
)
target_include_directories(${PROJECT_NAME}_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_server
  rclcpp
  rclcpp_action
  arm_cell_interfaces
)
target_link_libraries(${PROJECT_NAME}_server ${PROJECT_NAME}_core)

add_executable(motion_node src/motion_node.cpp src/motion_node_impl.cpp)
target_include_directories(motion_node PRIVATE include)
ament_target_dependencies(motion_node rclcpp rclcpp_action arm_cell_interfaces std_srvs)
target_link_libraries(motion_node ${PROJECT_NAME}_server ${PROJECT_NAME}_core)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_server motion_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  arm_cell_interfaces
  geometry_msgs
  moveit_ros_planning_interface
  rosgraph_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_motion_core tests/test_motion_core.cpp)
  target_link_libraries(test_motion_core ${PROJECT_NAME}_core)
  target_include_directories(test_motion_core PRIVATE include)
  ament_target_dependencies(test_motion_core arm_cell_interfaces)

  ament_add_gtest(
    test_motion_node_parameters tests/test_motion_node_parameters.cpp src/motion_node_impl.cpp)
  target_link_libraries(test_motion_node_parameters ${PROJECT_NAME}_server ${PROJECT_NAME}_core)
  target_include_directories(test_motion_node_parameters PRIVATE include)
  ament_target_dependencies(
    test_motion_node_parameters rclcpp rclcpp_action arm_cell_interfaces std_srvs)

  ament_add_gtest(test_motion_action_server tests/test_motion_action_server.cpp)
  target_link_libraries(test_motion_action_server ${PROJECT_NAME}_server ${PROJECT_NAME}_core)
  target_include_directories(test_motion_action_server PRIVATE include)
  ament_target_dependencies(test_motion_action_server rclcpp rclcpp_action arm_cell_interfaces)

  ament_add_gtest(test_direct_stop_path tests/test_direct_stop_path.cpp)
  target_link_libraries(test_direct_stop_path ${PROJECT_NAME}_core)
  target_include_directories(test_direct_stop_path PRIVATE include)
  ament_target_dependencies(test_direct_stop_path arm_cell_interfaces)

  ament_add_gtest(test_home_and_retract tests/test_home_and_retract.cpp)
  target_link_libraries(test_home_and_retract ${PROJECT_NAME}_core)
  target_include_directories(test_home_and_retract PRIVATE include)
  ament_target_dependencies(test_home_and_retract arm_cell_interfaces)

  ament_add_gtest(test_pick_place_lifecycle tests/test_pick_place_lifecycle.cpp)
  target_link_libraries(test_pick_place_lifecycle ${PROJECT_NAME}_core)
  target_include_directories(test_pick_place_lifecycle PRIVATE include)
  ament_target_dependencies(test_pick_place_lifecycle arm_cell_interfaces)

  ament_add_gtest(test_gripper_port tests/test_gripper_port.cpp)
  target_link_libraries(test_gripper_port ${PROJECT_NAME}_core)
  target_include_directories(test_gripper_port PRIVATE include)
  ament_target_dependencies(test_gripper_port arm_cell_interfaces)

  ament_add_gtest(test_isaac_gripper_backend tests/test_isaac_gripper_backend.cpp)
  target_link_libraries(test_isaac_gripper_backend ${PROJECT_NAME}_core)
  target_include_directories(test_isaac_gripper_backend PRIVATE include)
  ament_target_dependencies(test_isaac_gripper_backend arm_cell_interfaces)

  ament_add_gtest(test_isaac_ros_motion_transport tests/test_isaac_ros_motion_transport.cpp)
  target_link_libraries(test_isaac_ros_motion_transport ${PROJECT_NAME}_core)
  target_include_directories(test_isaac_ros_motion_transport PRIVATE include)
  ament_target_dependencies(
    test_isaac_ros_motion_transport rclcpp sensor_msgs arm_cell_interfaces tf2)

  ament_add_gtest(test_grasp_contact_policy tests/test_grasp_contact_policy.cpp)
  target_link_libraries(test_grasp_contact_policy ${PROJECT_NAME}_core)
  target_include_directories(test_grasp_contact_policy PRIVATE include)
  ament_target_dependencies(test_grasp_contact_policy moveit_msgs)

  ament_add_gtest(test_trajectory_validation tests/test_trajectory_validation.cpp)
  target_link_libraries(test_trajectory_validation ${PROJECT_NAME}_core)
  target_include_directories(test_trajectory_validation PRIVATE include)

  ament_add_gtest(
    test_executable_trajectory_finalize tests/test_executable_trajectory_finalize.cpp)
  target_link_libraries(test_executable_trajectory_finalize ${PROJECT_NAME}_core)
  target_include_directories(test_executable_trajectory_finalize PRIVATE include)
  ament_target_dependencies(test_trajectory_validation trajectory_msgs)

  ament_add_gtest(test_trajectory_sampler_worker tests/test_trajectory_sampler_worker.cpp)
  target_include_directories(test_trajectory_sampler_worker PRIVATE include)
  target_link_libraries(test_trajectory_sampler_worker arm_cell_motion_moveit2_core)
  ament_target_dependencies(test_trajectory_sampler_worker trajectory_msgs)

  ament_add_gtest(test_motion_profile_trace tests/test_motion_profile_trace.cpp)
  target_include_directories(test_motion_profile_trace PRIVATE include)
  target_link_libraries(test_motion_profile_trace ${PROJECT_NAME}_core)

  ament_add_gtest(test_trajectory_watchdog tests/test_trajectory_watchdog.cpp)
  target_include_directories(test_trajectory_watchdog PRIVATE include)
  target_link_libraries(test_trajectory_watchdog ${PROJECT_NAME}_core)

  ament_add_gtest(test_motion_progress_watchdog tests/test_motion_progress_watchdog.cpp)
  target_include_directories(test_motion_progress_watchdog PRIVATE include)
  target_link_libraries(test_motion_progress_watchdog ${PROJECT_NAME}_core)

  ament_add_gtest(test_final_target_hold tests/test_final_target_hold.cpp)
  target_include_directories(test_final_target_hold PRIVATE include)
  target_link_libraries(test_final_target_hold ${PROJECT_NAME}_core)

  ament_add_gtest(test_trajectory_settling_diagnostics tests/test_trajectory_settling_diagnostics.cpp)
  target_include_directories(test_trajectory_settling_diagnostics PRIVATE include)
  target_link_libraries(test_trajectory_settling_diagnostics ${PROJECT_NAME}_core)

  ament_add_gtest(test_approach_entry_acceptance tests/test_approach_entry_acceptance.cpp)
  target_include_directories(test_approach_entry_acceptance PRIVATE include)
  target_link_libraries(test_approach_entry_acceptance ${PROJECT_NAME}_core)
  ament_target_dependencies(test_approach_entry_acceptance geometry_msgs)

  ament_add_gtest(test_trajectory_continuity tests/test_trajectory_continuity.cpp)
  target_include_directories(test_trajectory_continuity PRIVATE include)
  target_link_libraries(test_trajectory_continuity ${PROJECT_NAME}_core)
  ament_target_dependencies(test_trajectory_continuity trajectory_msgs)

  ament_add_gtest(test_place_approach_ik_policy tests/test_place_approach_ik_policy.cpp)
  target_include_directories(test_place_approach_ik_policy PRIVATE include)
  target_link_libraries(test_place_approach_ik_policy ${PROJECT_NAME}_core)

  ament_add_gtest(test_place_approach_trajectory tests/test_place_approach_trajectory.cpp)
  target_include_directories(test_place_approach_trajectory PRIVATE include)
  target_link_libraries(test_place_approach_trajectory ${PROJECT_NAME}_core)
  ament_target_dependencies(test_place_approach_trajectory moveit_core moveit_msgs)

  ament_add_gtest(test_cartesian_manipulation tests/test_cartesian_manipulation.cpp)
  target_include_directories(test_cartesian_manipulation PRIVATE include src)
  target_link_libraries(test_cartesian_manipulation ${PROJECT_NAME}_core)
  ament_target_dependencies(
    test_cartesian_manipulation geometry_msgs trajectory_msgs arm_cell_interfaces)

  add_launch_test(tests/test_motion_capability_launch.py TIMEOUT 40)
  add_launch_test(tests/test_safety_motion_stop_integration.py TIMEOUT 60)
  add_launch_test(tests/test_safety_motion_stop_immediate.py TIMEOUT 60)
  add_launch_test(tests/test_safety_motion_stop_negative.py TIMEOUT 60)
endif()

ament_package()
