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

#include <robotiq_controllers/epick_controller.hpp>

namespace robotiq_controllers
{
enum CommandInterfaces : size_t
{
  REGULATE_GRIPPER_CMD = 0
};

constexpr auto kRegulateCommandInterface = "gripper_cmd/regulate";

controller_interface::InterfaceConfiguration EpickController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(kRegulateCommandInterface);
  return config;
}

controller_interface::InterfaceConfiguration EpickController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
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
    // Create a service to regulate the gripper.
    regulate_gripper_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
        "/regulate", [this](std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr resp) { this->regulate_gripper(req, resp); });
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
    regulate_gripper_srv_.reset();
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

bool EpickController::regulate_gripper(std_srvs::srv::SetBool::Request::SharedPtr request,
                                       [[maybe_unused]] std_srvs::srv::SetBool::Response::SharedPtr response)
{
  command_interfaces_[REGULATE_GRIPPER_CMD].set_value(request->data ? 1.0 : 0.0);

  // TODO: read the response.
  //  resp->success = command_interfaces_[1].get_value();

  //  while (command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].get_value() == ASYNC_WAITING) {
  //    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  //  }
  //  resp->success = command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].get_value();

  //  return resp->success;
  return true;
}

}  // namespace robotiq_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_controllers::EpickController, controller_interface::ControllerInterface)
