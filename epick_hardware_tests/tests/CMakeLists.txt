# Add support for GTest.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(epick_driver REQUIRED)

ament_lint_auto_find_test_dependencies()

# GMock throws an error if we don't switch off this option in tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-sign-conversion)
endif()

###############################################################################
# test_default_command_interface

ament_add_gmock(test_default_command_interface
    test_default_command_interface.cpp
    test_utils.hpp
)
target_include_directories(test_default_command_interface
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_default_command_interface epick_driver::epick_driver)
ament_target_dependencies(test_default_command_interface
    epick_driver
    pluginlib
    ros2_control_test_assets
)
