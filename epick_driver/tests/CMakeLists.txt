# Add support for GTest.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ros2_control_test_assets REQUIRED)

ament_lint_auto_find_test_dependencies()

# GMock throws an error if we don't switch off this option in tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-sign-conversion)
endif()

###############################################################################
# test_default_driver

ament_add_gmock(test_default_driver
    test_default_driver.cpp
    mock/mock_serial.hpp
)
target_include_directories(test_default_driver
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_default_driver epick_driver)

###############################################################################
# test_epick_gripper_hardware_interface

ament_add_gmock(test_epick_gripper_hardware_interface
    test_epick_gripper_hardware_interface.cpp
)
target_include_directories(test_epick_gripper_hardware_interface
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_epick_gripper_hardware_interface epick_driver)
ament_target_dependencies(test_epick_gripper_hardware_interface
    ros2_control_test_assets
)

###############################################################################
# test_default_serial_factory

ament_add_gmock(test_default_serial_factory
    test_default_serial_factory.cpp
    mock/mock_serial.hpp
)
target_include_directories(test_default_serial_factory
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_default_serial_factory epick_driver)

###############################################################################
# test_default_driver_factory

ament_add_gmock(test_default_driver_factory
    test_default_driver_factory.cpp
    mock/mock_driver.hpp
)
target_include_directories(test_default_driver_factory
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_default_driver_factory epick_driver)

###############################################################################
# test_data_utils

ament_add_gmock(test_data_utils
    test_data_utils.cpp
)
target_include_directories(test_data_utils
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_data_utils epick_driver)
