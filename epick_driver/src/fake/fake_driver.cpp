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

#include <epick_driver/fake/fake_driver.hpp>
#include <epick_driver/default_driver_utils.hpp>

#include <rclcpp/logging.hpp>

namespace epick_driver
{
const auto kLogger = rclcpp::get_logger("FakeDriver");

void FakeDriver::set_slave_address(uint8_t slave_address)
{
  slave_address_ = slave_address;
  RCLCPP_INFO(kLogger, "slave_address set to: %d", slave_address);
}

void FakeDriver::set_mode(const GripperMode gripper_mode)
{
  gripper_mode_ = gripper_mode;
  RCLCPP_INFO(kLogger, "mode set to: %s", default_driver_utils::gripper_mode_to_string(gripper_mode).c_str());
}

void FakeDriver::set_grip_max_vacuum_pressure(float vacuum_pressure)
{
  grip_max_vacuum_pressure_ = vacuum_pressure;
  RCLCPP_INFO(kLogger, "grip max vacuum pressure set to: %fkPa", vacuum_pressure);
}

void FakeDriver::set_grip_min_vacuum_pressure(float vacuum_pressure)
{
  grip_min_vacuum_pressure_ = vacuum_pressure;
  RCLCPP_INFO(kLogger, "grip min vacuum pressure set to: %fkPa", vacuum_pressure);
}

void FakeDriver::set_grip_timeout(std::chrono::milliseconds timeout)
{
  grip_timeout_ = timeout;
  RCLCPP_INFO(kLogger, "grip timeout set to: %ldms", timeout.count());
}

void FakeDriver::set_release_timeout(std::chrono::milliseconds timeout)
{
  release_timeout_ = timeout;
  RCLCPP_INFO(kLogger, "release timeout set to: %ldms", timeout.count());
}

bool FakeDriver::connect()
{
  connected_ = true;
  RCLCPP_INFO(kLogger, "Gripper connected.");
  return true;
}

void FakeDriver::disconnect()
{
  RCLCPP_INFO(kLogger, "Gripper disconnected.");
  connected_ = false;
}

void FakeDriver::activate()
{
  RCLCPP_INFO(kLogger, "Gripper activated.");
  activated_ = true;
}

void FakeDriver::deactivate()
{
  RCLCPP_INFO(kLogger, "Gripper deactivated.");
  activated_ = false;
}

void FakeDriver::grip()
{
  regulate_ = true;
  RCLCPP_INFO(kLogger, "Grip enable.");
}

void FakeDriver::release()
{
  regulate_ = false;
  RCLCPP_INFO(kLogger, "Grip released.");
}

GripperStatus FakeDriver::get_status()
{
  GripperStatus status;
  status.gripper_activation_action =
      activated_ ? GripperActivationAction::Activate : GripperActivationAction::ClearGripperFaultStatus;
  status.gripper_mode = gripper_mode_;
  status.gripper_regulate_action =
      regulate_ ? GripperRegulateAction::FollowRequestedVacuumParameters : GripperRegulateAction::StopVacuumGenerator;
  status.gripper_activation_status =
      activated_ ? GripperActivationStatus::GripperOperational : GripperActivationStatus::GripperNotActivated;
  status.gripper_fault_status = GripperFaultStatus::NoFault;
  status.actuator_status = regulate_ ? ActuatorStatus::Gripping : ActuatorStatus::PassiveReleasing;
  status.object_detection_status =
      regulate_ ? ObjectDetectionStatus::ObjectDetectedAtMaxPressure : ObjectDetectionStatus::NoObjectDetected;
  status.max_vacuum_pressure = grip_max_vacuum_pressure_;
  status.actual_vacuum_pressure = (grip_max_vacuum_pressure_ + grip_min_vacuum_pressure_) / 2;
  return status;
}
}  // namespace epick_driver
