cmake_minimum_required(VERSION 3.8)
project(arm_cell_motion)

execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  OUTPUT_VARIABLE ARM_CELL_BUILD_COMMIT_SHA
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
execute_process(
  COMMAND git status --porcelain
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
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
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(arm_cell_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/config/pick_approach_planning.xml")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/config/pick_approach_planning.xml" PICK_APPROACH_TREE_XML)
configure_file(src/pick_approach_tree_xml.hpp.in generated/pick_approach_tree_xml.hpp @ONLY)

add_library(${PROJECT_NAME}_core
  src/motion_core.cpp
  src/candidate_search.cpp
  src/isaac_motion_backend.cpp
  src/isaac_gripper_port.cpp
  src/isaac_ros_motion_transport.cpp
  src/trajectory_validation.cpp
  src/executable_trajectory_finalize.cpp
  src/trajectory_sampler.cpp
  src/trajectory_execution_worker.cpp
  src/motion_profile_trace.cpp
  src/trajectory_watchdog.cpp
  src/clock_diagnostic_capture.cpp
  src/motion_progress_watchdog.cpp
  src/trajectory_continuity.cpp
  src/motion_transport_joint_command.cpp
  src/place_approach_ik_policy.cpp
  src/place_approach_trajectory.cpp
  src/pick_grasp_ik.cpp
  src/pick_grasp_kinematics.cpp
  src/pick_grasp_contact.cpp
  src/finger_clearance.cpp
  src/occupancy_intersection.cpp
  src/grasp_collision_validator.cpp
  src/pick_state_collision.cpp
  src/pick_approach_ik.cpp
  src/pick_approach_planning_bt.cpp
  src/pick_planning_diagnostics.cpp
  src/occupancy_snapshot_renderer.cpp
  src/motion_path_planning.cpp
  src/place_object_pose_candidates.cpp
  src/cartesian_manipulation.cpp
  src/motion_execution_support.cpp
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
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  rosgraph_msgs
  unique_identifier_msgs
  tf2
  pluginlib
  behaviortree_cpp_v3
)
target_include_directories(${PROJECT_NAME}_core SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_core PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(${PROJECT_NAME}_core nlohmann_json::nlohmann_json ${OpenCV_LIBS})

add_library(${PROJECT_NAME}_runtime src/motion_node_impl.cpp)
target_include_directories(${PROJECT_NAME}_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_runtime rclcpp arm_cell_interfaces std_srvs)
target_link_libraries(${PROJECT_NAME}_runtime ${PROJECT_NAME}_core)

add_executable(motion_node src/motion_node.cpp)
target_include_directories(motion_node PRIVATE include)
ament_target_dependencies(motion_node rclcpp)
target_link_libraries(motion_node ${PROJECT_NAME}_runtime ${PROJECT_NAME}_core)

install(
  DIRECTORY include/
  DESTINATION include
)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()
install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_runtime
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS motion_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  arm_cell_interfaces
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  unique_identifier_msgs
  rosgraph_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_ros_planning REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_grasp_collision_validator tests/test_grasp_collision_validator.cpp)
  target_link_libraries(test_grasp_collision_validator ${PROJECT_NAME}_core)

  ament_add_gtest(test_finger_clearance tests/test_finger_clearance.cpp src/finger_clearance.cpp src/occupancy_intersection.cpp)
  target_include_directories(test_finger_clearance PRIVATE include)
  ament_target_dependencies(test_finger_clearance moveit_core)

  ament_add_gtest(test_pick_planning_diagnostics tests/test_pick_planning_diagnostics.cpp)
  target_link_libraries(test_pick_planning_diagnostics ${PROJECT_NAME}_core)

  ament_add_gtest(test_pick_approach_planning_bt tests/test_pick_approach_planning_bt.cpp)
  target_link_libraries(test_pick_approach_planning_bt ${PROJECT_NAME}_core)
  target_include_directories(test_pick_approach_planning_bt PRIVATE include)
  ament_target_dependencies(test_pick_approach_planning_bt behaviortree_cpp_v3)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_candidate_occupancy tests/test_candidate_occupancy.cpp)
  target_include_directories(test_candidate_occupancy PRIVATE include)
  ament_target_dependencies(test_candidate_occupancy arm_cell_interfaces moveit_msgs)

  ament_add_gtest(test_planning_scene_builder tests/test_planning_scene_builder.cpp)
  target_include_directories(test_planning_scene_builder PRIVATE include)
  ament_target_dependencies(
    test_planning_scene_builder arm_cell_interfaces moveit_core moveit_msgs std_msgs shape_msgs)

  ament_add_gtest(test_occupancy_snapshot_renderer tests/test_occupancy_snapshot_renderer.cpp)
  target_link_libraries(test_occupancy_snapshot_renderer ${PROJECT_NAME}_core)
  target_include_directories(test_occupancy_snapshot_renderer SYSTEM PRIVATE
    ${OpenCV_INCLUDE_DIRS})

  ament_add_gtest(test_pick_search_controller tests/test_pick_search_controller.cpp)
  target_include_directories(test_pick_search_controller PRIVATE include)

  ament_add_gtest(test_candidate_search tests/test_candidate_search.cpp)
  target_link_libraries(test_candidate_search ${PROJECT_NAME}_core)
  ament_add_gtest(test_vision_excluded_planning tests/test_vision_excluded_planning.cpp)
  target_include_directories(test_vision_excluded_planning PRIVATE include)
  ament_target_dependencies(
    test_vision_excluded_planning
    arm_cell_interfaces
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
    rclcpp)
  ament_add_gtest(test_motion_port tests/test_motion_port.cpp)
  target_link_libraries(test_motion_port ${PROJECT_NAME}_core)
  target_include_directories(test_motion_port PRIVATE include)
  ament_target_dependencies(test_motion_port arm_cell_interfaces sensor_msgs unique_identifier_msgs)
  ament_add_gtest(test_disturbance_motion tests/test_disturbance_motion.cpp)
  target_link_libraries(test_disturbance_motion arm_cell_motion_core)
  target_include_directories(test_disturbance_motion PRIVATE include)
  ament_target_dependencies(
    test_disturbance_motion arm_cell_interfaces geometry_msgs moveit_core moveit_msgs)

  ament_add_gtest(
    test_motion_node_parameters tests/test_motion_node_parameters.cpp src/motion_node_impl.cpp)
  target_link_libraries(test_motion_node_parameters ${PROJECT_NAME}_runtime ${PROJECT_NAME}_core)
  target_include_directories(test_motion_node_parameters PRIVATE include)
  ament_target_dependencies(
    test_motion_node_parameters rclcpp arm_cell_interfaces std_srvs)

  ament_add_gtest(test_direct_stop_path tests/test_direct_stop_path.cpp)
  target_link_libraries(test_direct_stop_path ${PROJECT_NAME}_core)
  target_include_directories(test_direct_stop_path PRIVATE include)
  ament_target_dependencies(test_direct_stop_path arm_cell_interfaces)

  ament_add_gtest(
    test_motion_planning_diagnostic_capture tests/test_motion_planning_diagnostic_capture.cpp)
  target_link_libraries(test_motion_planning_diagnostic_capture ${PROJECT_NAME}_core)
  target_include_directories(test_motion_planning_diagnostic_capture PRIVATE include)
  ament_target_dependencies(test_motion_planning_diagnostic_capture rclcpp)

  ament_add_gtest(test_pick_orientation_candidates tests/test_pick_orientation_candidates.cpp)
  target_link_libraries(test_pick_orientation_candidates ${PROJECT_NAME}_core)
  target_include_directories(test_pick_orientation_candidates PRIVATE include)
  ament_target_dependencies(test_pick_orientation_candidates arm_cell_interfaces)

  ament_add_gtest(test_pick_grasp_kinematics tests/test_pick_grasp_kinematics.cpp)
  target_link_libraries(test_pick_grasp_kinematics ${PROJECT_NAME}_core)
  target_include_directories(test_pick_grasp_kinematics PRIVATE include)

  ament_add_gtest(test_pick_grasp_contact tests/test_pick_grasp_contact.cpp)
  target_link_libraries(test_pick_grasp_contact ${PROJECT_NAME}_core)
  target_include_directories(test_pick_grasp_contact PRIVATE include)

  ament_add_gtest(test_box_grasp_geometry tests/test_box_grasp_geometry.cpp)
  target_include_directories(test_box_grasp_geometry PRIVATE include)
  ament_target_dependencies(test_box_grasp_geometry moveit_core)

  # Recovery scenarios each initialize a real MoveGroup client (3 s discovery
  # bound), in addition to the existing transport integration scenarios.
  ament_add_gtest(test_isaac_ros_motion_transport tests/test_isaac_ros_motion_transport.cpp
    TIMEOUT 180)
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
  target_link_libraries(test_trajectory_sampler_worker arm_cell_motion_core)
  ament_target_dependencies(test_trajectory_sampler_worker trajectory_msgs)

  ament_add_gtest(test_motion_profile_trace tests/test_motion_profile_trace.cpp)
  target_include_directories(test_motion_profile_trace PRIVATE include)
  target_link_libraries(test_motion_profile_trace ${PROJECT_NAME}_core)

  ament_add_gtest(test_trajectory_watchdog tests/test_trajectory_watchdog.cpp)
  target_include_directories(test_trajectory_watchdog PRIVATE include)
  target_link_libraries(test_trajectory_watchdog ${PROJECT_NAME}_core)

  ament_add_gtest(test_clock_diagnostic_capture tests/test_clock_diagnostic_capture.cpp)
  target_include_directories(test_clock_diagnostic_capture PRIVATE include)
  target_link_libraries(test_clock_diagnostic_capture ${PROJECT_NAME}_core)

  ament_add_gtest(test_motion_progress_watchdog tests/test_motion_progress_watchdog.cpp)
  target_include_directories(test_motion_progress_watchdog PRIVATE include)
  target_link_libraries(test_motion_progress_watchdog ${PROJECT_NAME}_core)


  ament_add_gtest(test_trajectory_continuity tests/test_trajectory_continuity.cpp)
  target_include_directories(test_trajectory_continuity PRIVATE include)
  target_link_libraries(test_trajectory_continuity ${PROJECT_NAME}_core)
  ament_target_dependencies(test_trajectory_continuity trajectory_msgs)

  ament_add_gtest(test_task_accuracy tests/test_task_accuracy.cpp)
  target_include_directories(test_task_accuracy PRIVATE include)
  ament_target_dependencies(test_task_accuracy moveit_core)

  ament_add_gtest(test_place_approach_ik_policy tests/test_place_approach_ik_policy.cpp)
  target_include_directories(test_place_approach_ik_policy PRIVATE include)
  target_link_libraries(test_place_approach_ik_policy ${PROJECT_NAME}_core)

  ament_add_gtest(test_place_approach_trajectory tests/test_place_approach_trajectory.cpp)
  target_include_directories(test_place_approach_trajectory PRIVATE include)
  target_link_libraries(test_place_approach_trajectory ${PROJECT_NAME}_core)
  ament_target_dependencies(
    test_place_approach_trajectory moveit_core moveit_msgs moveit_ros_planning_interface rclcpp)

  ament_add_gtest(test_cartesian_manipulation tests/test_cartesian_manipulation.cpp)
  target_include_directories(test_cartesian_manipulation PRIVATE include src)
  target_link_libraries(test_cartesian_manipulation ${PROJECT_NAME}_core)
  ament_target_dependencies(
    test_cartesian_manipulation geometry_msgs trajectory_msgs arm_cell_interfaces)

  add_launch_test(tests/test_motion_capability_launch.py TIMEOUT 40)
  add_launch_test(tests/test_safety_motion_stop_integration.py TIMEOUT 60)
endif()

ament_package()
