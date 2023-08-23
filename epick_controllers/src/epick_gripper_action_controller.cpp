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

#include <algorithm>
#include <epick_controllers/epick_gripper_action_controller.hpp>
#include <optional>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace
{
// auto findCommandInterface(const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces, const
// std::string& name)
// {
//   auto interface_it = std::find_if(command_interfaces.cbegin(), command_interfaces.cend(), [&name](const
//   hardware_interface::LoanedCommandInterface& interface){ return interface.get_interface_name() == name; }); if
//   (interface_it == command_interfaces.cend())
//   {
//     return std::nullopt;
//   }
//   return interface_it;
// }
}

namespace epick_controllers
{
enum CommandInterfaces : size_t
{
  REGULATE_GRIPPER_CMD = 0
};

enum StateInterfaces : size_t
{
  OBJECT_DETECTION_STATUS = 0
};

constexpr auto kRegulateCommandInterface = "gripper/regulate";
constexpr auto kIsRegulatingStateInterface = "gripper/regulate";
constexpr auto kObjectDetectionStateInterface = "gripper/object_detection_status";

constexpr auto kRegulateService = "/regulate";
constexpr auto kObjectDetectionStatusTopic = "/object_detection_status";

controller_interface::InterfaceConfiguration EpickGripperActionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(kRegulateCommandInterface);
  return config;
}

controller_interface::InterfaceConfiguration EpickGripperActionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(kIsRegulatingStateInterface, kObjectDetectionStateInterface);
  return config;
}

controller_interface::return_type EpickGripperActionController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                       [[maybe_unused]] const rclcpp::Duration& period)
{
  // double object_detection_status = state_interfaces_[OBJECT_DETECTION_STATUS].get_value();
  // auto object_detection_status_msg = std::make_shared<epick_msgs::msg::ObjectDetectionStatus>();

  // if (object_detection_status < 0.5)
  // {
  //   object_detection_status_msg->status = object_detection_status_msg->UNKNOWN;
  // }
  // else if (object_detection_status < 1.5)
  // {
  //   object_detection_status_msg->status = object_detection_status_msg->OBJECT_DETECTED_AT_MIN_PRESSURE;
  // }
  // else if (object_detection_status < 2.5)
  // {
  //   object_detection_status_msg->status = object_detection_status_msg->OBJECT_DETECTED_AT_MAX_PRESSURE;
  // }
  // else if (object_detection_status < 3.5)
  // {
  //   object_detection_status_msg->status = object_detection_status_msg->NO_OBJECT_DETECTED;
  // }
  // else
  // {
  //   object_detection_status_msg->status = object_detection_status_msg->UNKNOWN;
  // }

  // object_detection_status_pub_->publish(*object_detection_status_msg);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickGripperActionController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  auto regulate_command_interface_it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                                                    [](const hardware_interface::LoanedCommandInterface& interface) {
                                                      return interface.get_interface_name() == "regulate";
                                                    });
  if (regulate_command_interface_it == command_interfaces_.end())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  regulate_command_interface_ = *regulate_command_interface_it;

  auto is_regulating_state_interface_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                                                       [](const hardware_interface::LoanedStateInterface& interface) {
                                                         return interface.get_interface_name() == "regulate";
                                                       });
  if (is_regulating_state_interface_it == state_interfaces_.end())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  is_regulating_state_interface_ = *is_regulating_state_interface_it;

  commands_.regulate = regulate_command_interface_->get().get_value();

  commands_buffer_.initRT(commands_);

  pre_alloc_result_ = std::make_shared<GripperCommandAction::Result>();
  pre_alloc_result_->position = commands_.regulate;
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  action_server_ = rclcpp_action::create_server<GripperCommandAction>(
      get_node(), "~/gripper_cmd",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GripperCommandAction::Goal> goal) {
        return goal_callback(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) { return cancel_callback(goal_handle); },
      [this](std::shared_ptr<GoalHandle> goal_handle) { return accepted_callback(goal_handle); });

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickGripperActionController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // try
  // {
  //   regulate_gripper_srv_.reset();
  // }
  // catch (...)
  // {
  //   return LifecycleNodeInterface::CallbackReturn::ERROR;
  // }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EpickGripperActionController::on_init()
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// bool EpickGripperActionController::regulate_gripper(std_srvs::srv::SetBool::Request::SharedPtr request,
//                                        [[maybe_unused]] std_srvs::srv::SetBool::Response::SharedPtr response)
// {
//   // command_interfaces_[REGULATE_GRIPPER_CMD].set_value(request->data ? 1.0 : 0.0);

//   // TODO: read the response.
//   //  resp->success = command_interfaces_[1].get_value();

//   //  while (command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].get_value() == ASYNC_WAITING) {
//   //    std::this_thread::sleep_for(std::chrono::milliseconds(50));
//   //  }
//   //  resp->success = command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].get_value();

//   //  return resp->success;
//   return true;
// }

rclcpp_action::GoalResponse
EpickGripperActionController::goal_callback(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const GripperCommandAction::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
EpickGripperActionController::cancel_callback(const std::shared_ptr<GoalHandle> goal_handle)
{
}

void EpickGripperActionController::accepted_callback(std::shared_ptr<GoalHandle> goal_handle)
{
}

void EpickGripperActionController::preempt_active_goal()
{
}

}  // namespace epick_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(epick_controllers::EpickGripperActionController, controller_interface::ControllerInterface)
