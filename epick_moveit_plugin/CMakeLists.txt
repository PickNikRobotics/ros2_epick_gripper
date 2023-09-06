# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.8)
project(epick_moveit_plugin VERSION 0.0.1 LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_control_interface REQUIRED)
find_package(moveit_simple_controller_manager REQUIRED)
find_package(pluginlib REQUIRED)

add_library(epick_gripper_controller_allocator SHARED
  src/epick_gripper_controller_allocator.cpp
)
ament_target_dependencies(epick_gripper_controller_allocator PUBLIC
  pluginlib
  moveit_ros_control_interface
  moveit_simple_controller_manager
)

# Register the Epick gripper controller allocator plugin with MoveIt's ros_control interface
pluginlib_export_plugin_description_file(moveit_ros_control_interface moveit_ros_control_interface_plugins.xml)

# Install our library.
install(
  TARGETS epick_gripper_controller_allocator
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}      # bin
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)

ament_package()
