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
#include <epick_driver/default_driver_factory.hpp>

#include <epick_driver/fake/fake_driver.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include <chrono>

namespace epick_driver::test
{
using ::testing::Contains;
using ::testing::Eq;

// This factory will populate the injected driver with data read form the HardwareInfo.
class TestDriverFactory : public DefaultDriverFactory
{
public:
  explicit TestDriverFactory(std::unique_ptr<Driver> driver) : driver_{ std::move(driver) }
  {
  }

protected:
  std::unique_ptr<Driver> create_driver([[maybe_unused]] const hardware_interface::HardwareInfo& info) const override
  {
    return std::move(driver_);
  }

private:
  mutable std::unique_ptr<Driver> driver_;
};

/**
 * This method allow a busy wait on a given condition until a timeout is exceeded.
 * @param condition The condition to be met.
 * @param timeout The maximum waiting time.
 * @return True if the condition is met, false otherwise.
 */
bool wait_for_condition(std::function<bool()> condition, std::chrono::milliseconds timeout)
{
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout)
  {
    if (condition())
    {
      return true;  // Condition met
    }
    std::this_thread::yield();  // Yield the current thread to reduce busy-waiting overhead.
  }
  return false;  // Timeout reached
}

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
 * In this test we startup the hardware interface with a fake driver so we can
 * test read and write operations.
 */
TEST(TestEpickGripperHardwareInterface, regulate_interface)
{
  auto driver = std::make_unique<FakeDriver>();

  // We get our hands on the raw pointer to check expectations later.
  auto driver_handle = driver.get();

  auto hardware = std::make_unique<epick_driver::EpickGripperHardwareInterface>(
      std::make_unique<TestDriverFactory>(std::move(driver)));

  hardware_interface::HardwareInfo info{
    "EpickGripperHardwareInterface",
    "system",
    "epick_driver/EpickGripperHardwareInterface",
    {},  // parameters.
    {},  // joints.
    {},  // Sensors.
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

  // Claim the regulate interface.
  hardware_interface::LoanedCommandInterface regulate_cmd = rm.claim_command_interface("gripper_cmd/regulate");

  // Ask the gripper to grip.
  regulate_cmd.set_value(1.0);
  rm.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));

  auto gripper_gripping = [driver_handle]() {
    return driver_handle->get_status().gripper_regulate_action ==
           GripperRegulateAction::FollowRequestedVacuumParameters;
  };
  ASSERT_TRUE(wait_for_condition(gripper_gripping, std::chrono::milliseconds(500)))
      << "Timeout exceeded waiting for the gripper to grip.";

  // Ask the gripper to release.
  regulate_cmd.set_value(std::numeric_limits<double>::quiet_NaN());
  rm.write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));

  auto gripper_released = [driver_handle]() {
    return driver_handle->get_status().gripper_regulate_action == GripperRegulateAction::StopVacuumGenerator;
  };
  ASSERT_TRUE(wait_for_condition(gripper_released, std::chrono::milliseconds(500)))
      << "Timeout exceeded waiting for the gripper to release.";

  // Deactivate the hardware.
  rclcpp_lifecycle::State inactive_state{ lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                          hardware_interface::lifecycle_state_names::INACTIVE };
  rm.set_component_state("EpickGripperHardwareInterface", inactive_state);
}

}  // namespace epick_driver::test
