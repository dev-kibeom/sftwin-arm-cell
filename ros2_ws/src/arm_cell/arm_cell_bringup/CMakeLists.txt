cmake_minimum_required(VERSION 3.8)
project(arm_cell_bringup)
find_package(ament_cmake REQUIRED)
find_package(arm_cell_interfaces REQUIRED)
find_package(arm_cell_safety REQUIRED)
find_package(arm_cell_motion_moveit2 REQUIRED)
find_package(arm_cell_bt REQUIRED)
find_package(arm_cell_integration REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/register_pnp_fixture.py
    scripts/pnp_planning_scene.py
    scripts/pnp_planning_scene_runtime.py
    scripts/pnp_validation_preflight.py
    scripts/pnp_fixture_control.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_planning_launch tests/test_planning_launch.py)
  ament_add_pytest_test(test_camera_launch tests/test_camera_launch.py)
  ament_add_pytest_test(test_operational_profile_launch tests/test_operational_profile_launch.py)
  ament_add_pytest_test(test_production_launch tests/test_production_launch.py)
  ament_add_pytest_test(test_pnp_validation_profile tests/test_pnp_validation_profile.py)
  ament_add_pytest_test(test_register_pnp_fixture tests/test_register_pnp_fixture.py)
  ament_add_pytest_test(test_pnp_planning_scene tests/test_pnp_planning_scene.py)
  ament_add_pytest_test(test_pnp_planning_scene_runtime tests/test_pnp_planning_scene_runtime.py)
  ament_add_pytest_test(test_pnp_validation_preflight tests/test_pnp_validation_preflight.py)
  ament_add_pytest_test(test_node_lifetime_launch tests/test_node_lifetime_launch.py)
  ament_add_pytest_test(test_full_cell_synthetic tests/test_full_cell_synthetic.py)
  add_launch_test(tests/test_degraded_fault_scenarios_launch.py TIMEOUT 30)
  ament_add_pytest_test(test_m09_combined_launch tests/test_m09_combined_launch.py)
  add_launch_test(tests/test_m09_material_retry_launch.py TIMEOUT 40)
  add_launch_test(tests/test_production_material_flow_launch.py TIMEOUT 40)

  ament_add_gtest(test_full_cell_synthetic_cpp tests/test_full_cell_synthetic.cpp)
  target_link_libraries(test_full_cell_synthetic_cpp
    arm_cell_safety::arm_cell_safety_core
    arm_cell_motion_moveit2::arm_cell_motion_moveit2_core
    arm_cell_bt::arm_cell_orchestration_core
    yaml-cpp
  )
  ament_target_dependencies(test_full_cell_synthetic_cpp
    arm_cell_interfaces
    moveit_ros_planning_interface
    rclcpp
    sensor_msgs
    std_msgs
  )
endif()
ament_package()
