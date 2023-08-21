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

#include <epick_driver/epick_gripper_hardware_interface.hpp>

#include <epick_driver/default_driver_factory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <serial/serial.h>

#include <cmath>
#include <thread>

namespace epick_driver
{
const auto kLogger = rclcpp::get_logger("EpickGripperHardwareInterface");

constexpr auto kGripperGPIO = "gripper_cmd";
constexpr auto kRegulateCommandInterface = "regulate";

constexpr auto kCommandInterval = std::chrono::milliseconds{ 10 };

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
  try
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    return state_interfaces;
  }
  catch (const std::exception& ex)
  {
    set_state(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                      hardware_interface::lifecycle_state_names::UNCONFIGURED));
    return {};
  }
}

std::vector<hardware_interface::CommandInterface> EpickGripperHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  try
  {
    auto it = std::find_if(info_.gpios.begin(), info_.gpios.end(),
                           [](const auto& gpio) { return gpio.name == kGripperGPIO; });
    if (it != info_.gpios.end())
    {
      auto regulate_cmd = std::find_if(it->command_interfaces.begin(), it->command_interfaces.end(),
                                       [](const auto& cmd) { return cmd.name == kRegulateCommandInterface; });
      if (regulate_cmd != it->command_interfaces.end())
      {
        RCLCPP_DEBUG(kLogger, "export_command_interface %s", regulate_cmd->name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(it->name, regulate_cmd->name, &regulate_cmd_));
      }
      else
      {
        RCLCPP_ERROR(kLogger, "Command interface %s not found within %s.", kRegulateCommandInterface, kGripperGPIO);
      }
    }
    else
    {
      RCLCPP_ERROR(kLogger, "Command interface %s not found.", kGripperGPIO);
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
    driver_->deactivate();
    driver_->activate();

    // The following thread will be responsible for comminicating directly with the driver.
    communication_thread_is_running_.store(true);
    communication_thread_ = std::thread([this] { this->background_task(); });
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
  try
  {
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
  try
  {
    bool regulate = regulate_cmd_ >= 0.5;
    regulate_async_cmd_.store(regulate);
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
  // Read from and write to the gripper at 100 Hz.
  while (communication_thread_is_running_.load())
  {
    try
    {
      // Depending on the current gripper status decide to send or not a regulate command.
      auto status = driver_->get_status();
      bool regulate = regulate_async_cmd_.load();
      if (status.gripper_regulate_action == GripperRegulateAction::StopVacuumGenerator && regulate)
      {
        driver_->grip();
      }
      if (status.gripper_regulate_action == GripperRegulateAction::FollowRequestedVacuumParameters && !regulate)
      {
        driver_->release();
      }

      gripper_status_.store(driver_->get_status());
    }
    catch (serial::IOException& e)
    {
      RCLCPP_ERROR(kLogger, "Check Epick Gripper connection and restart drivers. Error: %s", e.what());
      communication_thread_is_running_.store(false);
    }

    std::this_thread::sleep_for(kCommandInterval);
  }
}
}  // namespace epick_driver

PLUGINLIB_EXPORT_CLASS(epick_driver::EpickGripperHardwareInterface, hardware_interface::SystemInterface)
