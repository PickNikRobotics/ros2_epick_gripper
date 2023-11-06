cmake_minimum_required(VERSION 3.8)
project(epick_moveit_studio LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(epick_msgs REQUIRED)
find_package(moveit_studio_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(epick_behaviors SHARED
  src/compare_epick_object_detection_status.cpp
  src/get_epick_object_detection_status.cpp
  src/behavior_loader.cpp
)
target_include_directories(epick_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
)
target_link_libraries(epick_behaviors PUBLIC moveit_studio_behavior_interface::async_behavior_base)
ament_target_dependencies(epick_behaviors PUBLIC
  epick_msgs
  moveit_studio_behavior_interface
  pluginlib
)

install(
  TARGETS
    epick_behaviors
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY
    config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  add_subdirectory(test)
endif()

pluginlib_export_plugin_description_file(moveit_studio_behavior_interface behavior_plugin_description.xml)

ament_export_include_directories(include)
ament_export_libraries(epick_behaviors)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(epick_msgs moveit_studio_behavior_interface pluginlib)
ament_package()
