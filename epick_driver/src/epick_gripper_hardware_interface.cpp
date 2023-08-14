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

#include "epick_driver/default_driver_factory.hpp"
#include "epick_driver/default_driver_utils.hpp"
#include "serial/serial.h"

#include <rclcpp/logging.hpp>

#include <cmath>

namespace epick_driver
{
const auto kLogger = rclcpp::get_logger("EpickGripperHardwareInterface");

auto kModeParamName = "mode";
auto kModeParamDefault = GripperMode::AutomaticMode;

auto kMaxVacuumPressureParamName = "max_vacuum_pressure";
auto kMaxVacuumPressureParamDefault = -100.0;  // kPA

auto kMinVacuumPressureParamName = "min_vacuum_pressure";
auto kMinVacuumPressureParamDefault = 0.0;  // kPA

auto kGripperTimeoutParamName = "gripper_timeout";
auto kGripperTimeoutParamDefault = 500;  // ms

EpickGripperHardwareInterface::EpickGripperHardwareInterface()
{
  driver_factory_ = std::make_unique<DefaultDriverFactory>();
}

// This constructor is use for testing only.
EpickGripperHardwareInterface::EpickGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory)
  : driver_factory_{ std::move(driver_factory) }
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

    GripperMode mode =
        info.hardware_parameters.count(kModeParamName) ?
            info.hardware_parameters.at(kModeParamName) == "advanced" ? GripperMode::AdvancedMode : kModeParamDefault :
            kModeParamDefault;
    RCLCPP_INFO(kLogger, "mode: %s", default_driver_utils::gripper_mode_to_string(mode).c_str());

    double max_vacuum_pressure = info.hardware_parameters.count(kMaxVacuumPressureParamName) ?
                                     std::stod(info.hardware_parameters.at(kMaxVacuumPressureParamName)) :
                                     kMaxVacuumPressureParamDefault;
    RCLCPP_INFO(kLogger, "%s: %fkPa", kMaxVacuumPressureParamName, max_vacuum_pressure);

    double min_vacuum_pressure = info.hardware_parameters.count(kMinVacuumPressureParamName) ?
                                     std::stod(info.hardware_parameters.at(kMinVacuumPressureParamName)) :
                                     kMinVacuumPressureParamDefault;
    RCLCPP_INFO(kLogger, "%s: %fkPa", kMinVacuumPressureParamName, min_vacuum_pressure);

    std::chrono::milliseconds gripper_timeout =
        info.hardware_parameters.count(kGripperTimeoutParamName) ?
            std::chrono::milliseconds(std::stoi(info.hardware_parameters.at(kGripperTimeoutParamName))) :
            std::chrono::milliseconds(kGripperTimeoutParamDefault);
    RCLCPP_INFO(kLogger, "%s: %ldms", kGripperTimeoutParamName, gripper_timeout.count());

    for (uint i = 0; i < info_.gpios.size(); i++)
    {
      auto gpio = info_.gpios[i];
    }

    driver_ = driver_factory_->create(info);
    driver_->set_mode(mode);
    driver_->set_max_vacuum_pressure(max_vacuum_pressure);
    driver_->set_min_vacuum_pressure(min_vacuum_pressure);
    driver_->set_gripper_timeout(gripper_timeout);

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
}

std::vector<hardware_interface::StateInterface> EpickGripperHardwareInterface::export_state_interfaces()
{
}

std::vector<hardware_interface::CommandInterface> EpickGripperHardwareInterface::export_command_interfaces()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickGripperHardwareInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_activate");
  try
  {
    driver_->deactivate();
    driver_->activate();
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

hardware_interface::return_type EpickGripperHardwareInterface::read(const rclcpp::Time& time,
                                                                    const rclcpp::Duration& period)
{
}

hardware_interface::return_type EpickGripperHardwareInterface::write(const rclcpp::Time& time,
                                                                     const rclcpp::Duration& period)
{
}
}  // namespace epick_driver
