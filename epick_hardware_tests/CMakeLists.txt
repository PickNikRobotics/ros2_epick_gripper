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
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
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
target_include_directories(activate
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(activate epick_driver::epick_driver)
ament_target_dependencies(activate epick_driver)

add_executable(get_status
    src/get_status.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
target_include_directories(get_status
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(get_status epick_driver::epick_driver)
ament_target_dependencies(get_status epick_driver)

add_executable(set_max_vacuum_pressure
    src/set_max_vacuum_pressure.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)
target_include_directories(set_max_vacuum_pressure
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(set_max_vacuum_pressure epick_driver::epick_driver)
ament_target_dependencies(set_max_vacuum_pressure epick_driver)

ament_package()
