cmake_minimum_required(VERSION 3.8)
project(arm_cell_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(arm_cell_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(arm_cell_orchestration_core
  src/cycle_coordinator.cpp
  src/recipe_repository.cpp
)
target_include_directories(arm_cell_orchestration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_cell_orchestration_core arm_cell_interfaces)
target_link_libraries(arm_cell_orchestration_core nlohmann_json::nlohmann_json)

add_executable(orchestration_node src/orchestration_node.cpp)
target_include_directories(orchestration_node PRIVATE include)
ament_target_dependencies(orchestration_node rclcpp rclcpp_action builtin_interfaces arm_cell_interfaces)
target_link_libraries(orchestration_node arm_cell_orchestration_core)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS arm_cell_orchestration_core orchestration_node
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
ament_export_dependencies(arm_cell_interfaces nlohmann_json)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(arm_cell_motion_moveit2 REQUIRED)
  ament_add_gtest(test_cycle_coordinator tests/test_cycle_coordinator.cpp)
  target_include_directories(test_cycle_coordinator PRIVATE include)
  target_compile_definitions(test_cycle_coordinator PRIVATE
    ARM_CELL_TEST_RECIPE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../arm_cell_bringup/config/recipes")
  target_link_libraries(
    test_cycle_coordinator
    arm_cell_orchestration_core
    arm_cell_motion_moveit2::arm_cell_motion_moveit2_core
  )
  ament_target_dependencies(
    test_cycle_coordinator arm_cell_interfaces builtin_interfaces)

  ament_add_gtest(test_recipe_repository tests/test_recipe_repository.cpp)
  target_include_directories(test_recipe_repository PRIVATE include)
  target_link_libraries(test_recipe_repository arm_cell_orchestration_core)
  ament_target_dependencies(test_recipe_repository arm_cell_interfaces)

  ament_add_gtest(test_controlled_recovery tests/test_controlled_recovery.cpp)
  target_include_directories(test_controlled_recovery PRIVATE include)
  target_link_libraries(test_controlled_recovery arm_cell_orchestration_core)
  ament_target_dependencies(test_controlled_recovery arm_cell_interfaces)

  ament_add_gtest(test_execute_cycle_action_terminal tests/test_execute_cycle_action_terminal.cpp)
  target_include_directories(test_execute_cycle_action_terminal PRIVATE include)
  ament_target_dependencies(
    test_execute_cycle_action_terminal
    rclcpp
    rclcpp_action
    arm_cell_interfaces
  )

  ament_add_gtest(test_recovery_wait_predicate tests/test_recovery_wait_predicate.cpp)
  target_include_directories(test_recovery_wait_predicate PRIVATE include)
  ament_target_dependencies(test_recovery_wait_predicate arm_cell_interfaces)

  ament_add_gtest(test_material_admission_gate tests/test_material_admission_gate.cpp)
  target_include_directories(test_material_admission_gate PRIVATE include)
  ament_target_dependencies(test_material_admission_gate arm_cell_interfaces)
endif()

ament_package()
