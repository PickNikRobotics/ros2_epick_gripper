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

#pragma once

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>

#include <memory>
#include <optional>

#include <control_msgs/action/gripper_command.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace epick_controllers
{
class EpickGripperActionController : public controller_interface::ControllerInterface
{
public:
  using GripperCommandAction = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<control_msgs::action::GripperCommand>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<std::shared_ptr<RealtimeGoalHandle>>;

  struct Commands
  {
    /**
     * @brief Represents the binary flag send to the gripper hardware interface to actuate the vacuum gripper.
     * @details If grip_cmd == 0.0, the gripper releases any held object. If grip_cmd == 1.0, the gripper activates its
     * vacuum pump and attempts to grasp an object.
     */
    double grip_cmd;
  };

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const GripperCommandAction::Goal> goal);

  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle> goal_handle);

  void accepted_callback(std::shared_ptr<GoalHandle> goal_handle);

  void preempt_active_goal();

  void set_hold_position();

  void check_for_success(const double current_regulate_state, const double current_regulate_command);

  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> regulate_command_interface_;

  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> is_regulating_state_interface_;

  std::shared_ptr<rclcpp_action::Server<GripperCommandAction>> action_server_;

  std::shared_ptr<rclcpp::TimerBase> goal_handle_timer_;

  RealtimeGoalHandleBuffer rt_active_goal_;

  GripperCommandAction::Result::SharedPtr pre_alloc_result_;

  realtime_tools::RealtimeBuffer<Commands> commands_buffer_;

  Commands commands_;
  Commands commands_rt_;
};
}  // namespace epick_controllers
