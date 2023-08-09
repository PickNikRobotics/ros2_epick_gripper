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

add_executable(activate_test src/activate_test.cpp)
target_include_directories(activate_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(activate_test epick_driver::epick_driver)
ament_target_dependencies(activate_test epick_driver)

add_executable(read_input_register_test src/read_input_register_test.cpp)
target_include_directories(read_input_register_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(read_input_register_test epick_driver::epick_driver)
ament_target_dependencies(read_input_register_test epick_driver)

ament_package()
