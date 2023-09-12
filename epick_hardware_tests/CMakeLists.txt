# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.8)
project(epick_hardware_tests VERSION 0.0.1 LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(epick_driver REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

add_executable(activate
    src/activate.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
ament_target_dependencies(activate epick_driver)

add_executable(deactivate
    src/deactivate.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
ament_target_dependencies(deactivate epick_driver)

add_executable(get_status
    src/get_status.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
ament_target_dependencies(get_status epick_driver)

add_executable(grip
    src/grip.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
ament_target_dependencies(grip epick_driver)

add_executable(release
    src/release.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
ament_target_dependencies(release epick_driver)

add_executable(break
    src/break.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
ament_target_dependencies(break epick_driver)

ament_package()
