# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.22.1)
project(epick_bringup VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
    config
    launch
    urdf
  DESTINATION share/epick_bringup
)

ament_package()
