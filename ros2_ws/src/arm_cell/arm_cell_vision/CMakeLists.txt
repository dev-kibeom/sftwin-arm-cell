cmake_minimum_required(VERSION 3.8)
project(arm_cell_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(arm_cell_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_library(vision_sensor_ingress
  src/vision_sensor_ingress.cpp
  src/clock_epoch_observer.cpp)
target_compile_features(vision_sensor_ingress PUBLIC cxx_std_17)
target_include_directories(vision_sensor_ingress PUBLIC include)
add_library(vision_ros_ingress
  src/vision_ros_adapter.cpp
  src/vision_ros_ingress.cpp)
target_compile_features(vision_ros_ingress PUBLIC cxx_std_17)
target_include_directories(vision_ros_ingress PUBLIC include)
target_link_libraries(vision_ros_ingress vision_sensor_ingress)
ament_target_dependencies(vision_ros_ingress rclcpp sensor_msgs message_filters rosgraph_msgs)
add_executable(vision_ros_ingress_node src/vision_ros_ingress_node.cpp)
target_link_libraries(vision_ros_ingress_node vision_ros_ingress)
ament_target_dependencies(vision_ros_ingress_node rclcpp)
install(TARGETS vision_ros_ingress_node DESTINATION lib/${PROJECT_NAME})

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
  ament_add_gtest(test_vision_sensor_ingress tests/test_vision_sensor_ingress.cpp)
  target_link_libraries(test_vision_sensor_ingress vision_sensor_ingress)
  ament_add_gtest(test_vision_ros_adapter tests/test_vision_ros_adapter.cpp)
  target_link_libraries(test_vision_ros_adapter vision_ros_ingress)
  ament_target_dependencies(test_vision_ros_adapter rclcpp sensor_msgs)
  set(M1C_FIXTURE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
  string(CONCAT M1C_FIXTURE_MANIFEST
    "${M1C_FIXTURE_ROOT}/infra/isaac_sim/scripts/m0609_cell/tests/fixtures/"
    "isaac_rgbd_m1c_verified_config_v2/manifest.json")
  target_compile_definitions(test_vision_ros_adapter PRIVATE
    M1C_FIXTURE_MANIFEST="${M1C_FIXTURE_MANIFEST}")
  add_executable(vision_ros_ingress_acceptance
    tests/acceptance/vision_ros_ingress_acceptance.cpp)
  target_include_directories(vision_ros_ingress_acceptance PRIVATE tests/acceptance)
  target_link_libraries(vision_ros_ingress_acceptance vision_ros_ingress)
  ament_target_dependencies(vision_ros_ingress_acceptance rclcpp)
  install(TARGETS vision_ros_ingress_acceptance DESTINATION lib/${PROJECT_NAME}/acceptance)
  ament_add_gtest(test_vision_ros_ingress_acceptance_logic
    tests/test_vision_ros_ingress_acceptance_logic.cpp)
  target_include_directories(test_vision_ros_ingress_acceptance_logic PRIVATE tests/acceptance)
  target_link_libraries(test_vision_ros_ingress_acceptance_logic vision_sensor_ingress)
endif()

ament_package()
