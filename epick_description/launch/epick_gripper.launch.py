# Copyright (c) 2023 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch file to start the Epick gripper."""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    """
    Launch function.

    Declare all parameters and extract their values, then start the nodes.
    'param: context' is the context object passed in by the launch framework.
    'param: *args' and 'param: **kwargs' are the arguments passed in by the
    launch file.
    """
    # Declare all parameters.
    description_package_param = LaunchConfiguration("description_package")
    description_file_param = LaunchConfiguration("description_file")
    controllers_config_file_param = LaunchConfiguration("controllers_file")

    # Extract all parameters' values.
    description_file = PathJoinSubstitution(
        [FindPackageShare(description_package_param), "urdf", description_file_param]
    ).perform(context)
    controllers_config_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package_param),
            "config",
            controllers_config_file_param,
        ]
    ).perform(context)

    robot_description_content = xacro.process_file(description_file).toxml()

    # The Controller Manager (CM) connects the controllers’ and hardware-abstraction
    # sides of the ros2_control framework. It also serves as the entry-point for users
    # through ROS services.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controllers_config_file,
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # This is a controller for the Robotiq Epick gripper.
    epick_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["epick_controller", "-c", "/controller_manager"],
    )

    # robot_state_publisher uses the URDF specified by the parameter robot_description
    # and the joint positions from the topic /joint_states to calculate the forward
    # kinematics of the robot and publish the results via tf.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen",
    )

    nodes_to_start = [
        controller_manager,
        epick_controller_spawner,
        robot_state_publisher_node,
    ]
    return nodes_to_start


def generate_launch_description():
    """
    Launch file entry point.

    A Python launch file is meant to help implement the markup based frontends like YAML
    and XML, and so it is declarative in nature rather than imperative. For this reason,
    it is not possible to directly access the content of LaunchConfiguration parameters,
    which are asyncio futures. To access the content of a LaunchConfiguration, we must
    provide a context by wrapping the initialization method into an OpaqueFunction.
    """
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="epick_description",
            description="Package containing all robot configuration files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="example.urdf.xacro",
            description="URDF/XACRO description file for the robot.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
        OpaqueFunction(function=launch_setup),
    ]

    return LaunchDescription(declared_arguments)
