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

#include <epick_controllers/epick_controller.hpp>

namespace epick_controllers
{
namespace command
{
enum CommandInterfaces : size_t
{
  GRIP_CMD = 0
};
}

namespace state
{
enum StateInterfaces : size_t
{
  GRIP_CMD = 0,
};
}

constexpr auto kGripCommandInterface = "gripper/grip_cmd";
constexpr auto kGripStateInterface = "gripper/grip_cmd";

constexpr auto kGripService = "/grip_cmd";

// If we use a service to set a double variable in a command interface, we want to wait until the same variable appears
// to have the same value in the corresponding state interface. We do not want to wait more than the give timeout.
constexpr auto kServiceTimeout = 2000;

// If we use a service to set a double variable in a command interface, we want to wait until the same variable appears
// to have the same value in the corresponding state interface. We check the value of the variable and then we sleep
// for this given amount of time before re-trying.
constexpr auto kBusyWaitingSleepTime = 50;

controller_interface::InterfaceConfiguration EpickController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(kGripCommandInterface);
  return config;
}

controller_interface::InterfaceConfiguration EpickController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(kGripStateInterface);
  return config;
}

controller_interface::return_type EpickController::update([[maybe_unused]] const rclcpp::Time& time,
                                                          [[maybe_unused]] const rclcpp::Duration& period)
{
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // Check command interfaces.
  if (command_interfaces_.size() != 1)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %d command interfaces, but got %zu.", 2,
                 command_interfaces_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  try
  {
    // Create a service to control the gripper.
    grip_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
        kGripService, [this](std_srvs::srv::SetBool::Request::SharedPtr req,
                             std_srvs::srv::SetBool::Response::SharedPtr resp) { this->grip_cmd(req, resp); });
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    grip_srv_.reset();
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EpickController::on_init()
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool EpickController::grip_cmd(std_srvs::srv::SetBool::Request::SharedPtr request,
                               [[maybe_unused]] std_srvs::srv::SetBool::Response::SharedPtr response)
{
  command_interfaces_[command::GRIP_CMD].set_value(request->data ? 1.0 : 0.0);

  auto start_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> timeout = std::chrono::milliseconds(kServiceTimeout);

  // We wait until the grip_cmd variable assumes the requested value.
  bool grip_cmd;
  do
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(kBusyWaitingSleepTime));
    grip_cmd = state_interfaces_[state::StateInterfaces::GRIP_CMD].get_value() >= 0.5;
  } while (request->data != grip_cmd && (std::chrono::steady_clock::now() - start_time) < timeout);

  if (request->data == grip_cmd)
  {
    response->success = true;
    response->message = "Regulate command succeeded.";
  }
  else
  {
    response->success = false;
    response->message = "The command failed.";
  }
  return response->success;
}

}  // namespace epick_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(epick_controllers::EpickController, controller_interface::ControllerInterface)
