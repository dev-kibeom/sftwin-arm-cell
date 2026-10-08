cmake_minimum_required(VERSION 3.8)
project(arm_cell_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(arm_cell_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RegisterFixedTarget.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs)

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
add_library(fixed_target_registry src/fixed_target_registry.cpp)
target_compile_features(fixed_target_registry PUBLIC cxx_std_17)
target_include_directories(fixed_target_registry PUBLIC include)
ament_target_dependencies(fixed_target_registry geometry_msgs)

add_library(detect_target_processor src/detect_target_processor.cpp)
target_sources(detect_target_processor PRIVATE src/rawpart_depth_profile.cpp)
target_compile_features(detect_target_processor PUBLIC cxx_std_17)
target_include_directories(detect_target_processor PUBLIC include)
ament_target_dependencies(detect_target_processor arm_cell_interfaces geometry_msgs sensor_msgs std_msgs tf2)
target_include_directories(detect_target_processor PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(detect_target_processor ${OpenCV_LIBS})
add_library(detect_target_service src/detect_target_service.cpp)
target_compile_features(detect_target_service PUBLIC cxx_std_17)
target_include_directories(detect_target_service PUBLIC include)
target_link_libraries(detect_target_service detect_target_processor vision_ros_ingress)
ament_target_dependencies(detect_target_service arm_cell_interfaces geometry_msgs std_msgs)
install(TARGETS
  vision_sensor_ingress
  vision_ros_ingress
  fixed_target_registry
  detect_target_processor
  detect_target_service
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
add_executable(fixed_detect_target_node src/fixed_detect_target_node.cpp)
target_link_libraries(fixed_detect_target_node fixed_target_registry)
ament_target_dependencies(fixed_detect_target_node rclcpp arm_cell_interfaces)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fixed_detect_target_node "${cpp_typesupport_target}")
install(TARGETS fixed_detect_target_node DESTINATION lib/${PROJECT_NAME})

add_executable(detect_target_node src/detect_target_node.cpp)
target_link_libraries(detect_target_node detect_target_service)
ament_target_dependencies(detect_target_node rclcpp tf2_ros arm_cell_interfaces)
install(TARGETS detect_target_node DESTINATION lib/${PROJECT_NAME})
add_executable(vision_ros_ingress_node src/vision_ros_ingress_node.cpp)
target_link_libraries(vision_ros_ingress_node vision_ros_ingress)
ament_target_dependencies(vision_ros_ingress_node rclcpp)
install(TARGETS vision_ros_ingress_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when a copyright and license is added to all source files
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
  ament_add_pytest_test(
    test_fixed_detect_target_ros tests/test_fixed_detect_target_ros.py)
  ament_add_gtest(test_fixed_target_registry tests/test_fixed_target_registry.cpp)
  target_link_libraries(test_fixed_target_registry fixed_target_registry)
  ament_target_dependencies(test_fixed_target_registry geometry_msgs)

  ament_add_gtest(test_detect_target_processor tests/test_detect_target_processor.cpp)
  target_link_libraries(test_detect_target_processor detect_target_processor)
  ament_add_gtest(test_rawpart_detect_target tests/test_rawpart_detect_target.cpp)
  target_link_libraries(test_rawpart_detect_target detect_target_processor ${OpenCV_LIBS})
  target_include_directories(test_rawpart_detect_target PRIVATE ${OpenCV_INCLUDE_DIRS})
  ament_add_gtest(test_detect_target_service tests/test_detect_target_service.cpp)
  target_link_libraries(test_detect_target_service detect_target_service)
  ament_target_dependencies(test_detect_target_service rclcpp arm_cell_interfaces)
endif()

ament_package()
