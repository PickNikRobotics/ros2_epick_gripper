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

#include <serial/serial.h>

#include <algorithm>
#include <cmath>
#include <iostream>

#include <epick_driver/default_driver.hpp>

#include <epick_driver/crc_utils.hpp>
#include <epick_driver/data_utils.hpp>
#include <epick_driver/default_driver_utils.hpp>
#include <epick_driver/driver_exception.hpp>

#include <rclcpp/logging.hpp>

// +-------------------------------+-------------------------------+
// | Gripper Input Registers       | Gripper Output Registers      |
// |-------------------------------+-------------------------------+
// | Address | Function            | Address | Function            |
// |---------+---------------------+---------+---------------------|
// | 0x03E8  | Action Register     | 0x07D0  | Gripper Status      |
// |         | Reserved            |         | Gripper Status Ext. |
// +---------+---------------------+---------+---------------------+
// | 0x03E9  | Reserved            | 0x07D1  | Fault status        |
// |         | Max Vacuum Pressure |         | Max Pressure        |
// +---------+---------------------+---------+---------------------+
// | 0x03EA  | Grip Timeout        | 0x07D2  | Actual Pressure     |
// |         | Min Vacuum Pressure |         | Reserved            |
// +---------+---------------------+---------+---------------------+
// | 0x03EB  | Reserved            | 0x07D3  | Reserved            |
// |         | Reserved            |         | Reserved            |
// +---------+---------------------+---------+---------------------+
// | 0x03EC  | Reserved            | 0x07D4  | Reserved            |
// |         | Reserved            |         | Reserved            |
// +---------+---------------------+---------+---------------------+
// | 0x03ED  | Reserved            | 0x07D5  | Reserved            |
// |         | Reserved            |         | Reserved            |
// +---------+---------------------+---------+---------------------+
// | 0x03EE  | Reserved            | 0x07D6  | Reserved            |
// |         | Reserved            |         | Reserved            |
// +---------+---------------------+---------+---------------------+
// | 0x03EF  | Reserved            | 0x07D7  | Reserved            |
// |         | Reserved            |         | Reserved            |
// +---------+---------------------+---------+---------------------+

namespace epick_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriver");

constexpr auto kAtmosphericPressure = 100.0f;  // kPa.

// If the gripper connection is not stable we may want to try sending the command again.
constexpr auto kMaxRetries = 5;

// The register containing the status of the gripper.
constexpr uint16_t kGripperStatusRegister = 0x07D0;

// The register containing all command requests.
constexpr uint16_t kActionRequestRegisterAddress = 0x03E8;

constexpr size_t kActivateResponseSize = 8;
constexpr size_t kDectivateResponseSize = 8;
constexpr size_t kGripResponseSize = 8;
constexpr size_t kReleaseResponseSize = 8;
constexpr size_t kGetStatusResponseSize = 11;

// Min and max timeout.
constexpr auto kMinTimeout = 0;      // ms
constexpr auto kMaxTimeout = 25500;  // ms

// Min and max absolute pressure.
constexpr auto kMinAbsolutePressure = 0.0f;    // kPa
constexpr auto kMaxAbsolutePressure = 255.0f;  // kPa

DefaultDriver::DefaultDriver(std::unique_ptr<Serial> serial) : serial_{ std::move(serial) }
{
}

std::vector<uint8_t> DefaultDriver::send(const std::vector<uint8_t>& request, size_t response_size) const
{
  std::vector<uint8_t> response;
  response.reserve(response_size);

  int retry_count = 0;
  while (retry_count < kMaxRetries)
  {
    try
    {
      serial_->write(request);
      response = serial_->read(response_size);
      break;
    }
    catch (const serial::IOException& e)
    {
      RCLCPP_DEBUG(kLogger, "Resending the command because the previous attempt (%d of %d) failed: %s", retry_count + 1,
                   kMaxRetries, e.what());
      retry_count++;
    }
  }

  if (retry_count == kMaxRetries)
  {
    RCLCPP_ERROR(kLogger, "Reached maximum retries. Operation failed.");
    return {};
  }

  return response;
}

bool DefaultDriver::connect()
{
  serial_->open();
  return serial_->is_open();
}

void DefaultDriver::disconnect()
{
  serial_->close();
}

void DefaultDriver::activate()
{
  RCLCPP_INFO(kLogger, "Activate...");

  uint8_t action_request_register = 0b00000000;
  default_driver_utils::set_gripper_activation_action(action_request_register, GripperActivationAction::Activate);
  default_driver_utils::set_gripper_mode(action_request_register, gripper_mode_);
  default_driver_utils::set_gripper_regulate_action(action_request_register,
                                                    GripperRegulateAction::FollowRequestedVacuumParameters);

  const uint8_t grip_max_absolute_pressure = static_cast<uint8_t>(kAtmosphericPressure);
  const uint8_t grip_min_absolute_pressure = static_cast<uint8_t>(kAtmosphericPressure);
  const auto timeout = static_cast<uint8_t>(
      std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 10>>>(
          std::clamp(grip_timeout_, std::chrono::milliseconds(0), std::chrono::milliseconds(kMaxTimeout)))
          .count());

  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress),
    data_utils::get_lsb(kActionRequestRegisterAddress),
    0x00,                        // Number of registers to write MSB.
    0x03,                        // Number of registers to write LSB.
    0x06,                        // Number of bytes to write.
    action_request_register,     // Action register.
    0x00,                        // Reserved.
    0x00,                        // Reserved.
    grip_max_absolute_pressure,  // Max absolute pressure.
    timeout,                     // Gripper Timeout.
    grip_min_absolute_pressure   // Min absolute pressure
  };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  auto response = send(request, kActivateResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to activate the gripper." };
  }
}

void DefaultDriver::deactivate()
{
  RCLCPP_INFO(kLogger, "Deactivate...");

  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress),
    data_utils::get_lsb(kActionRequestRegisterAddress),
    0x00,  // Number of registers to write MSB.
    0x03,  // Number of registers to write LSB.
    0x06,  // Number of bytes to write.
    0x00,  // Action register.
    0x00,  // Reserved.
    0x00,  // Reserved.
    0x00,  // Max absolute pressure.
    0x00,  // Gripper Timeout.
    0x00,  // Min absolute pressure
  };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  auto response = send(request, kDectivateResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to deactivate the gripper." };
  }
}

void DefaultDriver::grip()
{
  RCLCPP_INFO(kLogger, "Gripping...");

  uint8_t action_request_register = 0b00000000;
  default_driver_utils::set_gripper_activation_action(action_request_register, GripperActivationAction::Activate);
  default_driver_utils::set_gripper_mode(action_request_register, gripper_mode_);
  default_driver_utils::set_gripper_regulate_action(action_request_register,
                                                    GripperRegulateAction::FollowRequestedVacuumParameters);

  const uint8_t grip_max_absolute_pressure =
      static_cast<uint8_t>(gripper_mode_ == GripperMode::AdvancedMode ?
                               std::clamp(std::round(grip_max_vacuum_pressure_ + kAtmosphericPressure),
                                          kMinAbsolutePressure, kMaxAbsolutePressure) :
                               kMinAbsolutePressure);
  const uint8_t grip_min_absolute_pressure = static_cast<uint8_t>(std::clamp(
      std::round(grip_min_vacuum_pressure_ + kAtmosphericPressure), kMinAbsolutePressure, kMaxAbsolutePressure));
  const auto timeout = static_cast<uint8_t>(
      std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 10>>>(
          std::clamp(grip_timeout_, std::chrono::milliseconds(kMinTimeout), std::chrono::milliseconds(kMaxTimeout)))
          .count());

  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress),
    data_utils::get_lsb(kActionRequestRegisterAddress),
    0x00,                        // Number of registers to write MSB.
    0x03,                        // Number of registers to write LSB.
    0x06,                        // Number of bytes to write.
    action_request_register,     // Action register.
    0x00,                        // Reserved.
    0x00,                        // Reserved.
    grip_max_absolute_pressure,  // Grip max absolute pressure.
    timeout,                     // Gripper timeout (hundredths of a second).
    grip_min_absolute_pressure   // Min absolute pressure
  };

  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  auto response = send(request, kGripResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to grip." };
  }
}

void DefaultDriver::release()
{
  RCLCPP_INFO(kLogger, "Releasing...");

  uint8_t action_request_register = 0b00000000;
  default_driver_utils::set_gripper_activation_action(action_request_register, GripperActivationAction::Activate);
  default_driver_utils::set_gripper_mode(action_request_register, gripper_mode_);
  default_driver_utils::set_gripper_regulate_action(action_request_register,
                                                    GripperRegulateAction::FollowRequestedVacuumParameters);

  const uint8_t release_absolute_pressure = static_cast<uint8_t>(kMaxAbsolutePressure);
  const uint8_t grip_min_absolute_pressure = static_cast<uint8_t>(std::clamp(
      std::round(grip_min_vacuum_pressure_ + kAtmosphericPressure), kMinAbsolutePressure, kMaxAbsolutePressure));
  const auto timeout = static_cast<uint8_t>(
      std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 10>>>(
          std::clamp(release_timeout_, std::chrono::milliseconds(kMinTimeout), std::chrono::milliseconds(kMaxTimeout)))
          .count());

  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress),
    data_utils::get_lsb(kActionRequestRegisterAddress),
    0x00,                       // Number of registers to write MSB.
    0x03,                       // Number of registers to write LSB.
    0x06,                       // Number of bytes to write.
    action_request_register,    // Action register.
    0x00,                       // Reserved.
    0x00,                       // Reserved.
    release_absolute_pressure,  // Grip max absolute pressure.
    timeout,                    // Gripper timeout (hundredths of a second).
    grip_min_absolute_pressure  // Min absolute pressure
  };

  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  auto response = send(request, kReleaseResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to release." };
  }
}

void DefaultDriver::set_slave_address(uint8_t slave_address)
{
  slave_address_ = slave_address;
}

void DefaultDriver::set_mode(GripperMode gripper_mode)
{
  if (gripper_mode == GripperMode::Unknown)
  {
    RCLCPP_ERROR(kLogger, "Invalid gripper mode: %s",
                 default_driver_utils::gripper_mode_to_string(gripper_mode).c_str());
    return;
  }
  gripper_mode_ = gripper_mode;
}

void DefaultDriver::set_grip_max_vacuum_pressure(float vacuum_pressure)
{
  if (vacuum_pressure > 0)
  {
    RCLCPP_ERROR(kLogger, "Invalid grip max vacuum pressure: %f. Must be a value between -100kPa and 0kPa.",
                 vacuum_pressure);
    return;
  }
  grip_max_vacuum_pressure_ = vacuum_pressure;
}

void DefaultDriver::set_grip_min_vacuum_pressure(float vacuum_pressure)
{
  if (vacuum_pressure > 0)
  {
    RCLCPP_ERROR(kLogger, "Invalid grip min vacuum pressure: %f. Must be a value between -100kPa and 0kPa.",
                 vacuum_pressure);
    return;
  }
  grip_min_vacuum_pressure_ = vacuum_pressure;
}

void DefaultDriver::set_grip_timeout(std::chrono::milliseconds grip_timeout)
{
  grip_timeout_ = grip_timeout;
}

void DefaultDriver::set_release_timeout(std::chrono::milliseconds release_timeout)
{
  release_timeout_ = release_timeout;
}

GripperStatus DefaultDriver::get_status()
{
  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(default_driver_utils::FunctionCode::ReadInputRegisters),
    data_utils::get_msb(kGripperStatusRegister),
    data_utils::get_lsb(kGripperStatusRegister),
    0x00,  // Number of registers to read MSB
    0x03   // Number of registers to read LSB
  };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  auto response = send(request, kGetStatusResponseSize);
  if (response.empty())
  {
    throw DriverException{ "Failed to read the status." };
  }

  // The content of the requested registers starts from byte 3.

  GripperStatus status;
  status.gripper_activation_action = default_driver_utils::get_gripper_activation_action(response[3]);
  status.gripper_mode = default_driver_utils::get_gripper_mode(response[3]);
  status.gripper_regulate_action = default_driver_utils::get_gripper_regulate_action(response[3]);
  status.gripper_activation_status = default_driver_utils::get_gripper_activation_status(response[3]);
  status.object_detection_status = default_driver_utils::get_object_detection_status(response[3]);
  status.actuator_status = default_driver_utils::get_actuator_status(response[4]);
  status.gripper_fault_status = default_driver_utils::get_gripper_fault_status(response[5]);

  // The requested pressure level in kPa:
  status.max_vacuum_pressure = static_cast<float>(response[6]) - kAtmosphericPressure;

  // The actual pressure measured kPa:
  status.actual_vacuum_pressure = static_cast<float>(response[7]) - kAtmosphericPressure;

  return status;
}
}  // namespace epick_driver
