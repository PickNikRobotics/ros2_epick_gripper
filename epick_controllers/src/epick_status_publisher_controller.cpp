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

#include <epick_controllers/epick_status_publisher_controller.hpp>

namespace epick_controllers
{
namespace state
{
enum StateInterfaces : size_t
{
  OBJECT_DETECTION_STATUS = 0
};
}

constexpr auto kObjectDetectionStateInterface = "gripper/object_detection_status";

constexpr auto kObjectDetectionStatusTopic = "/object_detection_status";

controller_interface::InterfaceConfiguration EpickStatusPublisherController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration EpickStatusPublisherController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(kObjectDetectionStateInterface);
  return config;
}

controller_interface::return_type EpickStatusPublisherController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                         [[maybe_unused]] const rclcpp::Duration& period)
{
  double object_detection_status = state_interfaces_[state::OBJECT_DETECTION_STATUS].get_value();
  auto object_detection_status_msg = std::make_shared<epick_msgs::msg::ObjectDetectionStatus>();

  if (object_detection_status < 0.5)
  {
    object_detection_status_msg->status = object_detection_status_msg->UNKNOWN;
  }
  else if (object_detection_status < 1.5)
  {
    object_detection_status_msg->status = object_detection_status_msg->OBJECT_DETECTED_AT_MIN_PRESSURE;
  }
  else if (object_detection_status < 2.5)
  {
    object_detection_status_msg->status = object_detection_status_msg->OBJECT_DETECTED_AT_MAX_PRESSURE;
  }
  else if (object_detection_status < 3.5)
  {
    object_detection_status_msg->status = object_detection_status_msg->NO_OBJECT_DETECTED;
  }
  else
  {
    object_detection_status_msg->status = object_detection_status_msg->UNKNOWN;
  }

  object_detection_status_pub_->publish(*object_detection_status_msg);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickStatusPublisherController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    object_detection_status_pub_ =
        get_node()->create_publisher<epick_msgs::msg::ObjectDetectionStatus>(kObjectDetectionStatusTopic, 10);
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickStatusPublisherController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    object_detection_status_pub_.reset();
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EpickStatusPublisherController::on_init()
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace epick_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(epick_controllers::EpickStatusPublisherController, controller_interface::ControllerInterface)
