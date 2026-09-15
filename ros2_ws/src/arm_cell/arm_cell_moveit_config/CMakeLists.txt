cmake_minimum_required(VERSION 3.8)
project(arm_cell_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/load_static_scene.py
  scripts/generate_static_scene.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_moveit_configuration tests/test_moveit_configuration.py)
  ament_add_pytest_test(test_static_scene_loader tests/test_static_scene_loader.py)
endif()
ament_package()
