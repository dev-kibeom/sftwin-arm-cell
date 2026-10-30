cmake_minimum_required(VERSION 3.8)
project(arm_cell_vda)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(arm_cell_interfaces REQUIRED)

add_library(external_state_mock src/external_state_mock.cpp)
target_compile_features(external_state_mock PUBLIC cxx_std_17)
target_include_directories(external_state_mock PUBLIC include)
ament_target_dependencies(external_state_mock rclcpp arm_cell_interfaces)

add_library(material_delivery_mock src/material_delivery_mock.cpp)
target_compile_features(material_delivery_mock PUBLIC cxx_std_17)
target_include_directories(material_delivery_mock PUBLIC include)
ament_target_dependencies(material_delivery_mock arm_cell_interfaces)

add_executable(external_state_mock_node src/external_state_mock_node.cpp)
target_link_libraries(external_state_mock_node external_state_mock material_delivery_mock)
ament_target_dependencies(external_state_mock_node rclcpp std_srvs arm_cell_interfaces)
target_compile_options(external_state_mock_node PRIVATE -Wall -Wextra -Wpedantic)

install(
  TARGETS external_state_mock material_delivery_mock external_state_mock_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

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
  ament_add_gtest(test_external_state_mock tests/test_external_state_mock.cpp)
  target_link_libraries(test_external_state_mock external_state_mock)
  ament_target_dependencies(test_external_state_mock arm_cell_interfaces)
  ament_add_gtest(test_material_delivery_mock tests/test_material_delivery_mock.cpp)
  target_link_libraries(test_material_delivery_mock material_delivery_mock)
  target_include_directories(test_material_delivery_mock PRIVATE include)
  ament_target_dependencies(test_material_delivery_mock arm_cell_interfaces)
  add_launch_test(tests/test_external_state_mock_launch.py TIMEOUT 40)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
