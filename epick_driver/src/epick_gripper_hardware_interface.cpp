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

#include <serial/serial.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <thread>

#include <epick_driver/epick_gripper_hardware_interface.hpp>

#include <epick_driver/default_driver_factory.hpp>
#include <epick_driver/hardware_interface_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace epick_driver
{
const auto kLogger = rclcpp::get_logger("EpickGripperHardwareInterface");

constexpr auto kGripperPrefixName = "gripper";

constexpr auto kGripCommandInterfaceName = "grip_cmd";
constexpr auto kGripStateInterfacename = "grip_cmd";

constexpr auto kObjectDetectionStateInterfaceName = "object_detection_status";

constexpr auto kGripperCommsLoopPeriod = std::chrono::milliseconds{ 10 };

using epick_driver::hardware_interface_utils::get_gpios_command_interface;
using epick_driver::hardware_interface_utils::get_gpios_state_interface;
using epick_driver::hardware_interface_utils::get_joints_state_interface;
using epick_driver::hardware_interface_utils::is_false;
using epick_driver::hardware_interface_utils::is_true;

EpickGripperHardwareInterface::EpickGripperHardwareInterface()
{
  driver_factory_ = std::make_unique<DefaultDriverFactory>();
}

// This constructor is use for testing only.
EpickGripperHardwareInterface::EpickGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory)
  : driver_factory_{ std::move(driver_factory) }
{
}

EpickGripperHardwareInterface::~EpickGripperHardwareInterface()
{
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_DEBUG(kLogger, "on_init");
  try
  {
    // Store hardware info for later use.

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    for (uint i = 0; i < info_.gpios.size(); i++)
    {
      auto gpio = info_.gpios[i];
    }

    driver_ = driver_factory_->create(info);

    return CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot initialize the Robotiq EPick gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickGripperHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_configure");
  try
  {
    if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Open the serial port and handshake.
    bool connected = driver_->connect();
    if (!connected)
    {
      RCLCPP_ERROR(kLogger, "Cannot connect to the Robotiq EPick gripper");
      return CallbackReturn::ERROR;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot configure the Robotiq EPick gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EpickGripperHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  try
  {
    if (get_gpios_state_interface(kGripperPrefixName, kObjectDetectionStateInterfaceName, info_).has_value())
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          kGripperPrefixName, kObjectDetectionStateInterfaceName, &gripper_status_.object_detection_status));
    }
    else
    {
      RCLCPP_ERROR(kLogger, "State interface %s/%s not found.", kGripperPrefixName, kObjectDetectionStateInterfaceName);
    }
    if (get_gpios_state_interface(kGripperPrefixName, kGripStateInterfacename, info_).has_value())
    {
      state_interfaces.emplace_back(
          hardware_interface::StateInterface(kGripperPrefixName, kGripStateInterfacename, &gripper_status_.grip_cmd));
    }
    else
    {
      RCLCPP_ERROR(kLogger, "State interface %s/%s not found.", kGripperPrefixName, hardware_interface::HW_IF_POSITION);
    }

    // This joint state is optional.
    if (get_joints_state_interface(kGripperPrefixName, hardware_interface::HW_IF_POSITION, info_).has_value())
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          kGripperPrefixName, hardware_interface::HW_IF_POSITION, &gripper_status_.grip_cmd));
    }
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return {};
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EpickGripperHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  try
  {
    if (get_gpios_command_interface(kGripperPrefixName, kGripCommandInterfaceName, info_).has_value())
    {
      command_interfaces.emplace_back(
          hardware_interface::CommandInterface(kGripperPrefixName, kGripCommandInterfaceName, &gripper_cmds_.grip_cmd));
    }
    else
    {
      RCLCPP_ERROR(kLogger, "Command interface %s/%s not found.", kGripperPrefixName, kGripCommandInterfaceName);
    }
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return {};
  }
  return command_interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickGripperHardwareInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_activate");
  try
  {
    driver_->activate();

    // The following thread will be responsible for communicating directly with the driver.
    communication_thread_is_running_.store(true);
    communication_thread_ = std::thread([this] { this->background_task(); });
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to activate the Robotiq EPick gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "Robotiq EPick Gripper successfully activated!");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickGripperHardwareInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_deactivate");
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }
  try
  {
    driver_->deactivate();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to deactivate the Robotiq EPick gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Robotiq EPick Gripper successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EpickGripperHardwareInterface::read([[maybe_unused]] const rclcpp::Time& time,
                                                                    [[maybe_unused]] const rclcpp::Duration& period)
{
  // A state interface cannot be linked to atomic double values, only
  // double values. We must transfer the content of the atomic value, which is
  // set by the background thread, into the state interface value.
  try
  {
    gripper_status_.grip_cmd = safe_gripper_status_.grip_cmd.load();
    gripper_status_.object_detection_status = safe_gripper_status_.object_detection_status.load();
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EpickGripperHardwareInterface::write([[maybe_unused]] const rclcpp::Time& time,
                                                                     [[maybe_unused]] const rclcpp::Duration& period)
{
  // A command interface cannot be linked to atomic double values, only
  // double values. We must transfer the content of the command interface
  // double value into an atomic double so it can be read by the background
  // thread.
  try
  {
    safe_gripper_cmd_.grip_cmd.store(gripper_cmds_.grip_cmd);
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void EpickGripperHardwareInterface::background_task()
{
  // Read from and write to the gripper at a fixed rate set by kGripperCommsLoopPeriod.
  while (communication_thread_is_running_.load())
  {
    try
    {
      // Retrieve current status and update state interfaces
      const auto status = driver_->get_status();
      safe_gripper_status_.object_detection_status.store(
          default_driver_utils::object_detection_to_double(status.object_detection_status));

      // If the gripper or release command is successful, the gripper_cmd state
      // interface value will follow the gripper_cmd command interface value.
      const auto grip_cmd = safe_gripper_cmd_.grip_cmd.load();
      const auto grip_state = safe_gripper_status_.grip_cmd.load();

      if (is_false(grip_state) && is_true(grip_cmd))
      {
        driver_->grip();
        safe_gripper_status_.grip_cmd.store(grip_cmd);
      }
      else if (is_true(grip_state) && is_false(grip_cmd))
      {
        driver_->release();
        safe_gripper_status_.grip_cmd.store(grip_cmd);
      }
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(kLogger, "Error: %s", e.what());
    }
  }

  std::this_thread::sleep_for(kGripperCommsLoopPeriod);
}
}  // namespace epick_driver

PLUGINLIB_EXPORT_CLASS(epick_driver::EpickGripperHardwareInterface, hardware_interface::SystemInterface)
