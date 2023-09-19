# ROS2 Robotiq EPick Gripper

[![CI](https://github.com/PickNikRobotics/ros2_epick_gripper/actions/workflows/industrial_ci.yml/badge.svg)](https://github.com/PickNikRobotics/ros2_epick_gripper/actions/workflows/industrial_ci.yml)
[![Format](https://github.com/PickNikRobotics/ros2_epick_gripper/actions/workflows/ci-format.yml/badge.svg)](https://github.com/PickNikRobotics/ros2_epick_gripper/actions/workflows/ci-format.yml)
[![Linters](https://github.com/PickNikRobotics/ros2_epick_gripper/actions/workflows/ci-ros-lint.yml/badge.svg)](https://github.com/PickNikRobotics/ros2_epick_gripper/actions/workflows/ci-ros-lint.yml)

This repository contains the ROS 2 driver, controller, and description packages for working with a Robotiq EPick Gripper.

## Hardware Interface

The `epick_driver` package serves as a ROS 2 Hardware Interface for the Robotiq EPick Gripper, enabling direct interaction with the gripper hardware.

Below is a sample configuration that outlines various parameters for this interface.

```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="epick_driver_ros2_control" params="name">

    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>epick_driver/EpickGripperHardwareInterface</plugin>

       <!-- Serial connection parameters -->

        <param name="usb_port">/dev/ttyUSB0</param>
        <param name="baud_rate">115200</param>
        <param name="timeout">0.2</param>

        <!-- Gripper parameters -->

        <!-- Set use_dummy to true to connect to a dummy driver for testing purposes. -->
        <param name="use_dummy">false</param>

        <!-- The address of the gripper. -->
        <param name="slave_address">0x9</param>

        <!-- The gripper operation mode: AutomaticMode or AdvancedMode. -->
        <param name="mode">AdvancedMode</param>

        <!-- The following parameters are only required for the AdvancedMode. -->
        <param name="grip_max_vacuum_pressure">-60</param>
        <param name="grip_min_vacuum_pressure">-10</param>
        <param name="grip_timeout">25.0</param>
        <param name="release_timeout">2.0</param>

      </hardware>

      <gpio name="gripper">
        <!--
          Command interface to control the gripper:
          1.0 = grip
          0.0 = release
        -->
        <command_interface name="grip_cmd"/>

        <!--
          State interface that follow the value of the command interface:
          1.0 = successful grip
          0.0 = successful release
        -->
        <state_interface name="grip_cmd"/>

        <!--
          Return the object detection status:
          0.0 = unknown
          1.0 = object detected at minimum pressure
          2.0 = object detected at maximum pressure
          3.0 = no object detected
        -->
        <state_interface name="object_detection_status"/>
      </gpio>

      <!--
        This is optional configuration if you want to publish the state of the
        gripper as a joint state interface.
      -->
      <joint name="gripper">
        <!--
          State interface that follows the value of the gripper/grip_cmd
          command interface:
          1.0 = successful grip
          0.0 = successful release
        -->
        <state_interface name="position"/>
      </joint>

    </ros2_control>

</xacro:macro>
</robot>
```

The hardware interface exposes a command interface labeled `grip_cmd` for gripping and releasing operations. Assigning the value of 1.0 to this command interface initiates the gripping action, while a value of 0.0 triggers the release mechanism.

A parallel state interface named `grip_cmd` reflects the status of the performed command.

The interface also offers an `object_detection_status` state interface to provide real-time object detection feedback during the grip operation.

## Controller

The `epick_controllers` package establishes a ROS 2 Controller that communicates with the hardware interface through ROS 2 messages.

The controller exposes a ROS 2 service `/grip_cmd` that accepts a boolean request to control the gripper. A true request initiates gripping, whereas a false request triggers release.

For monitoring purposes, the controller also provides a `/object_detection_status` topic that returns one of the following object detection states:

- UNKNOWN;
- OBJECT_DETECTED_AT_MIN_PRESSURE;
- OBJECT_DETECTED_AT_MAX_PRESSURE;
- NO_OBJECT_DETECTED.

No specialized configuration is needed for the controller. Here is a sample `controllers.yaml` file for reference:

```
controller_manager:
  ros__parameters:
    update_rate: 30 # Hz

    epick_controller:
      type: epick_controllers/EpickController
```

## Test commands

The `epick_hardware_tests` package includes a set of terminal-based commands designed for hardware validation. The available commands are as follows:

- `activate`: Establishes a connection to the gripper and activates it.
- `deactivate`: Connects to the gripper and deactivates it.
- `get_status`: Obtains the current status of the gripper after activation.
- `grip`: Initiates the gripping action.
- `release`: Executes the release operation.
