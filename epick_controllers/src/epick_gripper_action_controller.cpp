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

#include <epick_controllers/epick_gripper_action_controller.hpp>

#include <algorithm>
#include <chrono>
#include <limits>

#include <rclcpp/logging.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace
{
/**
 * @brief Convert a double to a bool using the same logic the EpickGripperHardwareInterface uses to convert
 * the boolean gripper command and state values into doubles when storing them in the ros2_control interfaces.
 *
 * @param value Value to convert
 * @return Returns true if the value is greater than or equal to 0.5, and false if it is less than 0.5.
 */
bool toBool(double value)
{
  return value >= 0.5;
}
}  // namespace

namespace epick_controllers
{
constexpr auto kGripCommandInterface = "gripper/grip_cmd";
constexpr auto kGripStateInterface = "gripper/grip_cmd";

constexpr auto kActionName = "~/gripper_cmd";

constexpr auto kCheckGoalHandlePeriod = std::chrono::milliseconds{ 100 };

controller_interface::InterfaceConfiguration EpickGripperActionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(kGripCommandInterface);
  return config;
}

controller_interface::InterfaceConfiguration EpickGripperActionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(kGripStateInterface);
  return config;
}

controller_interface::return_type EpickGripperActionController::update([[maybe_unused]] const rclcpp::Time& time,
                                                                       [[maybe_unused]] const rclcpp::Duration& period)
{
  commands_rt_ = *(commands_buffer_.readFromRT());

  const double current_command = commands_rt_.grip_cmd;
  const double current_state = is_regulating_state_interface_->get().get_value();

  check_for_success(current_state, current_command);

  regulate_command_interface_->get().set_value(current_command);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EpickGripperActionController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  auto regulate_command_interface_it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                                                    [](const hardware_interface::LoanedCommandInterface& interface) {
                                                      return interface.get_interface_name() == "grip_cmd";
                                                    });
  if (regulate_command_interface_it == command_interfaces_.end())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  regulate_command_interface_ = *regulate_command_interface_it;

  auto is_regulating_state_interface_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                                                       [](const hardware_interface::LoanedStateInterface& interface) {
                                                         return interface.get_interface_name() == "grip_cmd";
                                                       });
  if (is_regulating_state_interface_it == state_interfaces_.end())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  is_regulating_state_interface_ = *is_regulating_state_interface_it;

  commands_.grip_cmd = regulate_command_interface_->get().get_value();

  commands_buffer_.initRT(commands_);

  pre_alloc_result_ = std::make_shared<GripperCommandAction::Result>();
  pre_alloc_result_->position = commands_.grip_cmd;
  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  action_server_ = rclcpp_action::create_server<GripperCommandAction>(
      get_node(), kActionName,
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
  regulate_command_interface_ = std::nullopt;
  is_regulating_state_interface_ = std::nullopt;

  release_interfaces();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EpickGripperActionController::on_init()
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse
EpickGripperActionController::goal_callback([[maybe_unused]] const rclcpp_action::GoalUUID& uuid,
                                            [[maybe_unused]] std::shared_ptr<const GripperCommandAction::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
EpickGripperActionController::cancel_callback(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    // Enter hold current position mode
    set_hold_position();

    RCLCPP_INFO(get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<GripperCommandAction::Result>();
    active_goal->setCanceled(action_res);
    // Reset current goal
    rt_active_goal_.writeFromNonRT(std::shared_ptr<RealtimeGoalHandle>());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void EpickGripperActionController::accepted_callback(std::shared_ptr<GoalHandle> goal_handle)
{
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);

  const auto& goal = goal_handle->get_goal();
  commands_.grip_cmd = goal->command.position;
  commands_buffer_.writeFromNonRT(commands_);

  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;

  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Replace previous timer with new one to handle the new action goal
  goal_handle_timer_.reset();
  goal_handle_timer_ =
      get_node()->create_wall_timer(kCheckGoalHandlePeriod, std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

void EpickGripperActionController::preempt_active_goal()
{
  // Cancels the currently active goal
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    // Marks the current goal as canceled
    active_goal->setCanceled(std::make_shared<GripperCommandAction::Result>());
    rt_active_goal_.writeFromNonRT(std::shared_ptr<RealtimeGoalHandle>());
  }
}

void EpickGripperActionController::set_hold_position()
{
  commands_.grip_cmd = regulate_command_interface_->get().get_value();
  commands_buffer_.writeFromNonRT(commands_);
}

void EpickGripperActionController::check_for_success(const double current_regulate_state,
                                                     const double current_regulate_command)
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (!active_goal)
  {
    return;
  }

  // The action has succeeded if the gripper's state becomes equal to the commanded value.
  if (toBool(current_regulate_command) == toBool(current_regulate_state))
  {
    RCLCPP_INFO(get_node()->get_logger(), "success!");

    pre_alloc_result_->position = current_regulate_state;
    pre_alloc_result_->reached_goal = true;
    pre_alloc_result_->stalled = false;

    active_goal->setSucceeded(pre_alloc_result_);
    rt_active_goal_.writeFromNonRT(std::shared_ptr<RealtimeGoalHandle>());
  }
}
}  // namespace epick_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(epick_controllers::EpickGripperActionController, controller_interface::ControllerInterface)
