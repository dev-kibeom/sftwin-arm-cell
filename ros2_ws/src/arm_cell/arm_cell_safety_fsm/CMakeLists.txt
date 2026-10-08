cmake_minimum_required(VERSION 3.8)
project(arm_cell_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_cell_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/safety_core.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core arm_cell_interfaces)

add_executable(safety_node src/safety_node.cpp)
target_include_directories(safety_node PRIVATE include)
ament_target_dependencies(safety_node rclcpp arm_cell_interfaces)
target_link_libraries(safety_node ${PROJECT_NAME}_core)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}_core safety_node
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
ament_export_dependencies(arm_cell_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_safety_core tests/test_safety_core.cpp)
  target_link_libraries(test_safety_core ${PROJECT_NAME}_core)
  target_include_directories(test_safety_core PRIVATE include)
  ament_target_dependencies(test_safety_core arm_cell_interfaces)

  ament_add_gtest(test_safety_input_tracker tests/test_safety_input_tracker.cpp)
  target_link_libraries(test_safety_input_tracker ${PROJECT_NAME}_core)
  target_include_directories(test_safety_input_tracker PRIVATE include)
  ament_target_dependencies(test_safety_input_tracker arm_cell_interfaces)

  ament_add_gtest(test_safety_stop_coordination tests/test_safety_stop_coordination.cpp)
  target_link_libraries(test_safety_stop_coordination ${PROJECT_NAME}_core)
  target_include_directories(test_safety_stop_coordination PRIVATE include)
  ament_target_dependencies(test_safety_stop_coordination arm_cell_interfaces)
endif()

ament_package()
