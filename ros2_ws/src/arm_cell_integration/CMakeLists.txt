cmake_minimum_required(VERSION 3.8)
project(arm_cell_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_cell_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(material_handoff_core src/material_handoff_coordinator.cpp)
target_compile_features(material_handoff_core PUBLIC cxx_std_17)
target_include_directories(material_handoff_core PUBLIC include)
ament_target_dependencies(material_handoff_core arm_cell_interfaces)

add_executable(material_handoff_node src/material_handoff_node.cpp)
target_compile_features(material_handoff_node PRIVATE cxx_std_17)
target_link_libraries(material_handoff_node material_handoff_core)
ament_target_dependencies(material_handoff_node rclcpp arm_cell_interfaces)

install(TARGETS material_handoff_core material_handoff_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(tests/test_material_handoff_launch.py TIMEOUT 30)
  add_launch_test(tests/test_material_request_rejection_launch.py TIMEOUT 30)
  ament_add_gtest(test_material_handoff_coordinator tests/test_material_handoff_coordinator.cpp)
  target_include_directories(test_material_handoff_coordinator PRIVATE include)
  target_link_libraries(test_material_handoff_coordinator material_handoff_core)
  ament_target_dependencies(test_material_handoff_coordinator arm_cell_interfaces)
endif()

ament_package()
