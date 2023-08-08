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

#include "epick_driver/epick_gripper_hardware_interface.hpp"

#include "epick_driver/default_command_interface_factory.hpp"
#include "serial/serial.h"

#include <rclcpp/logging.hpp>

namespace epick_driver
{
const auto kLogger = rclcpp::get_logger("EpickGripperHardwareInterface");

EpickGripperHardwareInterface::EpickGripperHardwareInterface()
{
  command_interface_factory_ = std::make_unique<DefaultCommandInterfaceFactory>();
}

// This constructor is use for testing only.
EpickGripperHardwareInterface::EpickGripperHardwareInterface(
    std::unique_ptr<CommandInterfaceFactory> command_interface_factory)
  : command_interface_factory_{ std::move(command_interface_factory) }
{
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

    // TODO: initialize all required structures.

    command_interface_ = command_interface_factory_->create(info);

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
    bool connected = command_interface_->connect();
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
}

std::vector<hardware_interface::StateInterface> EpickGripperHardwareInterface::export_state_interfaces()
{
  // TODO: create a join with a value that can be 1 (gripper closed) or 2 (gripper open).
}

std::vector<hardware_interface::CommandInterface> EpickGripperHardwareInterface::export_command_interfaces()
{
  // TODO: create a join with a value that can be 1 (gripper closed) or 2 (gripper open).
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickGripperHardwareInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_activate");
  try
  {
    command_interface_->deactivate();
    command_interface_->activate();
  }
  catch (const serial::IOException& e)
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
  try
  {
    command_interface_->deactivate();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to deactivate the Robotiq EPick gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Robotiq EPick Gripper successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EpickGripperHardwareInterface::read(const rclcpp::Time& time,
                                                                    const rclcpp::Duration& period)
{
}

hardware_interface::return_type EpickGripperHardwareInterface::write(const rclcpp::Time& time,
                                                                     const rclcpp::Duration& period)
{
}
}  // namespace epick_driver
