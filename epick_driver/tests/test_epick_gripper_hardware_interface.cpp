// Copyright (c) 2023 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <epick_driver/epick_gripper_hardware_interface.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

namespace epick_driver::test
{
using ::testing::Contains;
using ::testing::Eq;
/**
 * This test generates a minimal xacro robot configuration and loads the
 * hardware interface plugin.
 */
TEST(TestEpickGripperHardwareInterface, load_urdf)
{
  std::string urdf_control_ =
      R"(
         <ros2_control name="EpickGripperHArdwareInterface" type="system">
           <hardware>
             <plugin>epick_driver/EpickGripperHardwareInterface</plugin>
             <param name="usb_port">/dev/whatever</param>
             <param name="baudrate">9600</param>
             <param name="timeout">500</param>
           </hardware>
           <gpio name="gripper_cmd">
               <command_interface name="regulate"/>
           </gpio>
         </ros2_control>
       )";

  auto urdf = ros2_control_test_assets::urdf_head + urdf_control_ + ros2_control_test_assets::urdf_tail;
  hardware_interface::ResourceManager rm(urdf);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
}

/**
 * This test generates a minimal xacro robot configuration and loads the
 * hardware interface plugin.
 */
TEST(TestEpickGripperHardwareInterface, load_dummy_driver)
{
  auto hardware = std::make_unique<epick_driver::EpickGripperHardwareInterface>();

  hardware_interface::HardwareInfo info{
    "EpickGripperHardwareInterface",
    "system",
    "epick_driver/EpickGripperHardwareInterface",
    { { "use_dummy", "true" } },  // parameters.
    {},                           // joints.
    {},                           // Sensors.
    // GPIOs
    { { "gripper_cmd", "GPIO", { { "regulate", "", "", "", "double", 1 } }, {}, { {} } } },
    {},  // Transmission.
    ""   // original xml.
  };

  // Load the component.
  hardware_interface::ResourceManager rm;
  rm.import_component(std::move(hardware), info);

  // Connect the hardware.
  rclcpp_lifecycle::State active_state{ lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                        hardware_interface::lifecycle_state_names::ACTIVE };
  rm.set_component_state("EpickGripperHardwareInterface", active_state);

  EXPECT_THAT(rm.command_interface_keys(), Contains(Eq("gripper_cmd/regulate")));

  // Write velocity values.
  hardware_interface::LoanedCommandInterface regulate_cmd = rm.claim_command_interface("gripper_cmd/regulate");
  regulate_cmd.set_value(1.0);

  // Invoke a write command.
  const rclcpp::Time time;
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0);
  rm.write(time, period);

  // We must disconnect the hardware because there is a background thread
  // running that must be stopped nicely.
  rclcpp_lifecycle::State inactive_state{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                          hardware_interface::lifecycle_state_names::INACTIVE };
  rm.set_component_state("EpickGripperHardwareInterface", inactive_state);
}

}  // namespace epick_driver::test
