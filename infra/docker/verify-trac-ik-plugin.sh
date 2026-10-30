#!/usr/bin/env bash
set -euo pipefail

plugin_prefix=$(ros2 pkg prefix trac_ik_kinematics_plugin)
description_prefix=$(ros2 pkg prefix arm_cell_description)
config_prefix=$(ros2 pkg prefix arm_cell_moveit_config)
plugin_library="$plugin_prefix/lib/libtrac_ik_kinematics_plugin.so"
plugin_description="$plugin_prefix/share/trac_ik_kinematics_plugin/trac_ik_kinematics_description.xml"

test -f "$plugin_library"
grep -Fq 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin' "$plugin_description"

probe_dir=$(mktemp -d)
trap 'rm -rf "$probe_dir"' EXIT

xacro "$description_prefix/share/arm_cell_description/urdf/m0609_robotiq_2f85.urdf.xacro" \
  > "$probe_dir/arm.urdf"
cp "$config_prefix/share/arm_cell_moveit_config/config/m0609_robotiq_2f85.srdf" \
  "$probe_dir/arm.srdf"

cat > "$probe_dir/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.8)
project(trac_ik_runtime_probe LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
add_executable(trac_ik_runtime_probe main.cpp)
target_compile_features(trac_ik_runtime_probe PRIVATE cxx_std_17)
ament_target_dependencies(trac_ik_runtime_probe moveit_core pluginlib rclcpp srdfdom urdf)
CMAKE

cat > "$probe_dir/main.cpp" <<'CPP'
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

#include <moveit/robot_model/robot_model.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

std::string read_file(const char * path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error(std::string("cannot read ") + path);
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto urdf = urdf::parseURDF(read_file(argv[1]));
    if (!urdf) {
      throw std::runtime_error("URDF parse failed");
    }
    auto srdf = std::make_shared<srdf::Model>();
    if (!srdf->initString(*urdf, read_file(argv[2]))) {
      throw std::runtime_error("SRDF parse failed");
    }
    moveit::core::RobotModel model(urdf, srdf);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter(
      "robot_description_kinematics.arm.solve_type", "Distance")});
    auto node = std::make_shared<rclcpp::Node>("trac_ik_runtime_probe", options);
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader(
      "moveit_core", "kinematics::KinematicsBase");
    auto plugin = loader.createSharedInstance(
      "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin");
    if (!plugin->initialize(node, model, "arm", "base_link", {"sf_grasp_tcp"}, 0.005)) {
      throw std::runtime_error("MoveIt TRAC-IK initialize() returned false");
    }
    std::cout << "PASS: MoveIt loaded TRAC-IK and initialized arm -> sf_grasp_tcp "
                 "with solve_type=Distance"
              << std::endl;
  } catch (const std::exception & exception) {
    std::cerr << "FAIL: " << exception.what() << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
CPP

cmake -S "$probe_dir" -B "$probe_dir/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$probe_dir/build" --parallel 2
"$probe_dir/build/trac_ik_runtime_probe" "$probe_dir/arm.urdf" "$probe_dir/arm.srdf"
