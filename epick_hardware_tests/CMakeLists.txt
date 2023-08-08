cmake_minimum_required(VERSION 3.22.1)
project(epick_hardware_tests VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

###############################################################################
# TESTS

include(CTest)
if (BUILD_TESTING)
    add_subdirectory(tests)
endif()

ament_package()
