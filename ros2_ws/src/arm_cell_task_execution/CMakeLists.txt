cmake_minimum_required(VERSION 3.8)
project(arm_cell_task_execution)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(arm_cell_interfaces REQUIRED)
find_package(arm_cell_motion REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/motion_bt_executor.cpp
  src/disturbance_execution.cpp
  src/pick_candidate_search.cpp
  src/pick_execution.cpp
  src/pick_execution_record.cpp
  src/pick_recovery.cpp
  src/place_execution.cpp
  src/place_recovery.cpp
  src/held_object_return.cpp
  src/object_state_tracker.cpp
  src/task_execution_support.cpp
  src/task_executor.cpp
  src/execute_pnp_action_server.cpp
  src/execute_disturb_action_server.cpp
  src/legacy_execute_task_action_server.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  ament_index_cpp
  arm_cell_interfaces
  arm_cell_motion
  behaviortree_cpp_v3
  geometry_msgs
  rclcpp
  rclcpp_action
  unique_identifier_msgs
)
target_link_libraries(${PROJECT_NAME}_core arm_cell_motion::arm_cell_motion_core nlohmann_json::nlohmann_json)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(TARGETS ${PROJECT_NAME}_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(EXPORT export_${PROJECT_NAME} DESTINATION share/${PROJECT_NAME}/cmake)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  arm_cell_interfaces arm_cell_motion behaviortree_cpp_v3 geometry_msgs
  rclcpp rclcpp_action unique_identifier_msgs)

add_executable(task_execution_node src/task_execution_node.cpp)
target_include_directories(task_execution_node PRIVATE include)
ament_target_dependencies(
  task_execution_node rclcpp rcl_interfaces rclcpp_action arm_cell_interfaces std_srvs)
target_link_libraries(task_execution_node ${PROJECT_NAME}_core arm_cell_motion::arm_cell_motion_runtime)
install(TARGETS task_execution_node RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_motion_plan_execute_action tests/test_motion_plan_execute_action.cpp)
  target_link_libraries(test_motion_plan_execute_action ${PROJECT_NAME}_core)
  ament_target_dependencies(
    test_motion_plan_execute_action arm_cell_interfaces behaviortree_cpp_v3)

  ament_add_gtest(
    test_task_execution_strategy tests/test_task_execution_strategy.cpp)
  target_link_libraries(test_task_execution_strategy ${PROJECT_NAME}_core)
  ament_target_dependencies(test_task_execution_strategy arm_cell_interfaces)

  ament_add_gtest(test_disturbance_execution tests/test_disturbance_execution.cpp)
  target_link_libraries(test_disturbance_execution ${PROJECT_NAME}_core)
  ament_target_dependencies(test_disturbance_execution arm_cell_interfaces)

  ament_add_gtest(test_pick_place_lifecycle tests/test_pick_place_lifecycle.cpp)
  target_link_libraries(test_pick_place_lifecycle ${PROJECT_NAME}_core)
  ament_target_dependencies(test_pick_place_lifecycle arm_cell_interfaces)

  ament_add_gtest(test_execution_module_boundaries tests/test_execution_module_boundaries.cpp)
  target_link_libraries(test_execution_module_boundaries ${PROJECT_NAME}_core)
  ament_target_dependencies(test_execution_module_boundaries arm_cell_interfaces behaviortree_cpp_v3)

  ament_add_gtest(test_candidate_search_control tests/test_candidate_search_control.cpp)
  target_link_libraries(test_candidate_search_control ${PROJECT_NAME}_core)
  ament_target_dependencies(test_candidate_search_control behaviortree_cpp_v3)

  foreach(test_name test_gripper_port test_home_and_retract test_isaac_gripper_backend)
    ament_add_gtest(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME}_core)
    ament_target_dependencies(${test_name} arm_cell_interfaces)
  endforeach()

  ament_add_gtest(
    test_execute_disturb_action_server tests/test_execute_disturb_action_server.cpp)
  target_link_libraries(test_execute_disturb_action_server ${PROJECT_NAME}_core)
  ament_target_dependencies(
    test_execute_disturb_action_server arm_cell_interfaces rclcpp rclcpp_action)

  ament_add_gtest(
    test_legacy_execute_task_action_server tests/test_legacy_execute_task_action_server.cpp)
  target_link_libraries(test_legacy_execute_task_action_server ${PROJECT_NAME}_core)
  ament_target_dependencies(
    test_legacy_execute_task_action_server arm_cell_interfaces rclcpp rclcpp_action)
endif()

ament_package()
