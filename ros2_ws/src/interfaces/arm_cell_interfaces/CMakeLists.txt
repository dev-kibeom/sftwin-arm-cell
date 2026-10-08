cmake_minimum_required(VERSION 3.8)
project(arm_cell_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

set(msg_files
  "msg/AMRDockingState.msg"
  "msg/DetectTargetResultCode.msg"
  "msg/MissionExitReason.msg"
  "msg/MissionPhase.msg"
  "msg/MaterialHandoffState.msg"
  "msg/MaterialReadiness.msg"
  "msg/MotionCapability.msg"
  "msg/MotionStatus.msg"
  "msg/MotionTaskPhase.msg"
  "msg/MotionTaskResultCode.msg"
  "msg/MotionTaskType.msg"
  "msg/PackMLState.msg"
  "msg/SafetyCauseSet.msg"
  "msg/SafetyHardwareState.msg"
  "msg/SafetyState.msg"
  "msg/StopMode.msg"
  "msg/VisionDiagnostic.msg"
)

set(srv_files
  "srv/DetectTarget.srv"
  "srv/RequestMaterial.srv"
  "srv/RequestMaterialSupply.srv"
  "srv/ResetSafety.srv"
  "srv/StopMotion.srv"
)

set(action_files
  "action/ExecuteCycle.action"
  "action/ExecuteTask.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
    unique_identifier_msgs
)

if(BUILD_TESTING)
  add_executable(interface_contract_smoke tests/interface_contract_smoke.cpp)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(interface_contract_smoke "${cpp_typesupport_target}")
  add_test(NAME interface_contract_smoke COMMAND interface_contract_smoke)

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
