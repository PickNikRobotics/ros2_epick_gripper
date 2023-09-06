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

#include <atomic>
#include <thread>
#include <memory>
#include <vector>

#include <epick_driver/default_driver_utils.hpp>
#include <epick_driver/driver.hpp>
#include <epick_driver/driver_factory.hpp>
#include <epick_driver/visibility_control.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>

#include "visibility_control.hpp"

namespace epick_driver
{

// We use this structure to hold our command interface values.
struct Command
{
  double grip_cmd;
};

// We use this structure to hold a thread-safe copy of our command interface values.
struct SafeCommand
{
  std::atomic<double> grip_cmd;
};

// We use this structure to hold our state interface values.
struct State
{
  double grip_cmd;
  double object_detection_status;
};

// We use this structure to hold a thread-safe copy of our state interface values.
struct SafeState
{
  std::atomic<double> grip_cmd;
  std::atomic<double> object_detection_status;
};

class EpickGripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  EPICK_DRIVER_PUBLIC
  EpickGripperHardwareInterface();

  EPICK_DRIVER_PUBLIC
  ~EpickGripperHardwareInterface();

  /**
   * Constructor with given command interface. This method is used for testing.
   * @param driver_factory The interface to send commands and queries to
   * the hardware.
   */
  explicit EpickGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory);

  /**
   * Defines aliases and static functions for using the Class with shared_ptrs.
   * With such definitions, the control manager can instantiate the class with
   * auto hardware_interface = StepitHardware::make_shared();
   */
  RCLCPP_SHARED_PTR_DEFINITIONS(EpickGripperHardwareInterface)

  /**
   * Initialization of the hardware interface from data parsed from the
   * robot's URDF.
   * @param hardware_info Structure with data from URDF.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  EPICK_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * Connect to the hardware.
   * @param previous_state The previous state.
   * @returns CallbackReturn::SUCCESS if required data are provided and can be
   * parsed or CallbackReturn::ERROR if any error happens or data are missing.
   */
  EPICK_DRIVER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method exposes position and velocity of joints for reading.
   */
  EPICK_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * This method exposes the joints targets for writing.
   */
  EPICK_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * This method is invoked when the hardware is connected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  EPICK_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * This method is invoked when the hardware is disconnected.
   * @param previous_state Unconfigured, Inactive, Active or Finalized.
   * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
   */
  EPICK_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * Read data from the hardware.
   */
  EPICK_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * Write data to hardware.
   */
  EPICK_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Interface to interact with the hardware using the serial port.
  std::unique_ptr<Driver> driver_;

  // Factory to create the interface to interact with the hardware using the serial port.
  std::unique_ptr<DriverFactory> driver_factory_;

  // We use a thread to read/write to the driver so that we dont block the hardware_interface read/write.
  std::thread communication_thread_;
  std::atomic<bool> communication_thread_is_running_;
  void background_task();

  // This stores the double values of our GPIO command interfaces.
  Command gripper_cmds_{ default_driver_utils::regulate_action_to_double(GripperRegulateAction::StopVacuumGenerator) };

  // This stores thread-safe copies of our GPIO command interfaces values.
  // This is required because a command interface cannot be linked to an atomic double.
  SafeCommand safe_gripper_cmd_{ default_driver_utils::regulate_action_to_double(
      GripperRegulateAction::StopVacuumGenerator) };

  // This stores the double values of our GPIO state interfaces.
  State gripper_status_{ default_driver_utils::regulate_action_to_double(GripperRegulateAction::StopVacuumGenerator),
                         default_driver_utils::object_detection_to_double(ObjectDetectionStatus::Unknown) };

  // This stores thread-safe copies of our GPIO state interfaces values.
  // This is required because a command interface cannot be linked to an atomic double.
  SafeState safe_gripper_status_{ default_driver_utils::regulate_action_to_double(
                                      GripperRegulateAction::StopVacuumGenerator),
                                  default_driver_utils::object_detection_to_double(ObjectDetectionStatus::Unknown) };
};
}  // namespace epick_driver
