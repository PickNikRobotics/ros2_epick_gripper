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

#include "epick_driver/default_driver.hpp"

#include "epick_driver/crc_utils.hpp"
#include "epick_driver/data_utils.hpp"
#include "epick_driver/driver_utils.hpp"

#include "serial/serial.h"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

// |-------------------------------+-------------------------------+
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
constexpr uint16_t kGripperStatusRegister = 0x07D0;
constexpr uint16_t kActionRequestRegisterAddress = 0x03E8;

// Robot output/Gripper input registers: 0x03E8 to 0x03EF
// Robot input/Gripper output registers: 0x07D0 to 0x07D7

// The response to a write command consists of:
// - slave ID (1 byte)
// - function code (1 byte)
// - address of the first register that was written (2 bytes)
// - number of registers written (2 bytes)
// - CRC (2 bytes)
constexpr int kWriteResponseSize = 8;

const auto kLogger = rclcpp::get_logger("DefaultDriver");

/**
 * Sets bits in a register based on a bitmask and a set of bits.
 * @param reg Initial register value.
 * @param bitmask Mask that indicates which bits in the register should be modified.
 *        A '1' in a bit position indicates that the corresponding bit in the register
 *        will be modified, and a '0' means it will remain unchanged.
 * @param bits Bits to be set in the register. Only the bits that are '1' in the bitmask
 *        will be set in the register. Other bits will be ignored.
 */
void set_bits(uint8_t& reg, uint8_t bitmask, uint8_t bits)
{
  reg &= ~bitmask;
  reg |= (bits & bitmask);
}

DefaultDriver::DefaultDriver(std::unique_ptr<Serial> serial_interface, uint8_t slave_address)
  : serial_interface_{
    std::move(serial_interface),
  }, slave_address_{slave_address}
{
}

bool DefaultDriver::connect()
{
  serial_interface_->open();
  return serial_interface_->is_open();
}

void DefaultDriver::disconnect()
{
  serial_interface_->close();
}

void DefaultDriver::activate()
{
  const uint8_t max_absolute_pressure =
      static_cast<uint8_t>(std::clamp(std::round(max_vacuum_pressure_ + 100), 0.0f, 255.0f));

  const uint8_t min_absolute_pressure =
      static_cast<uint8_t>(std::clamp(std::round(min_vacuum_pressure_ + 100), 0.0f, 255.0f));

  std::chrono::milliseconds clamped_gripper_timeout =
      std::clamp(gripper_timeout_, std::chrono::milliseconds(0), std::chrono::milliseconds(25500));
  auto timeout_in_hundredths = static_cast<uint8_t>(
      std::chrono::duration_cast<std::chrono::duration<int, std::ratio<1, 100>>>(clamped_gripper_timeout).count());

  uint8_t action_register = 0b00000001;  // Activate action request.
  if (gripper_mode_ == GripperMode::AdvancedMode)
  {
    set_bits(action_register, driver_utils::gMOD_mask, 0b00000010);
  }
  else
  {
    set_bits(action_register, driver_utils::gMOD_mask, 0b00000000);
  }

  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress),
    data_utils::get_lsb(kActionRequestRegisterAddress),
    0x00,                   // Number of registers to write MSB.
    0x03,                   // Number of registers to write LSB.
    0x06,                   // Number of bytes to write.
    action_register,        // Action register.
    0x00,                   // Reserved.
    0x00,                   // Reserved.
    max_absolute_pressure,  // Max absolute pressure.
    timeout_in_hundredths,  // Gripper Timeout.
    min_absolute_pressure   // Min absolute pressure
  };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));
  try
  {
    serial_interface_->write(request);
    auto response = serial_interface_->read(kWriteResponseSize);
  }
  catch (const serial::IOException& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to activate the gripper: %s", e.what());
    throw;
  }
}

void DefaultDriver::deactivate()
{
  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress),
    data_utils::get_lsb(kActionRequestRegisterAddress),
    0x00,  // Number of registers to write MSB.
    0x03,  // Number of registers to write LSB.
    0x06,  // Number of bytes to write.
    0x00,  // Register 1 MSB.
    0x00,  // Register 1 LSB.
    0x00,  // Register 2 MSB.
    0x00,  // Register 2 LSB.
    0x00,  // Register 3 MSB.
    0x00,  // Register 3 LSB.

  };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  try
  {
    serial_interface_->write(request);
    auto response = serial_interface_->read(kWriteResponseSize);
  }
  catch (const serial::IOException& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to deactivate the gripper: %s", e.what());
    throw;
  }
}

void DefaultDriver::grip()
{
}

void DefaultDriver::release()
{
}

void DefaultDriver::set_mode(const GripperMode gripper_mode)
{
  if (gripper_mode == GripperMode::Unknown)
  {
    RCLCPP_ERROR(kLogger, "Invalid gripper mode: %s", driver_utils::gripper_mode_to_string(gripper_mode).c_str());
    return;
  }
  gripper_mode_ = gripper_mode;
}

void DefaultDriver::set_max_vacuum_pressure(const float& vacuum_pressure)
{
  max_vacuum_pressure_ = vacuum_pressure;
}

void DefaultDriver::set_min_vacuum_pressure(const float& vacuum_pressure)
{
  min_vacuum_pressure_ = vacuum_pressure;
}

void DefaultDriver::set_gripper_timeout(std::chrono::milliseconds gripper_timeout)
{
  gripper_timeout_ = gripper_timeout;
}

GripperStatus DefaultDriver::get_status()
{
  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(driver_utils::FunctionCode::ReadInputRegisters),
    data_utils::get_msb(kGripperStatusRegister),
    data_utils::get_lsb(kGripperStatusRegister),
    0x00,  // Number of registers to read MSB
    0x03   // Number of registers to read LSB
  };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  std::vector<uint8_t> response;
  try
  {
    serial_interface_->write(request);
    response = serial_interface_->read(kWriteResponseSize);
  }
  catch (const serial::IOException& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to read the gripper status: %s", e.what());
    throw;
  }

  // The content of the requested registers starts from byte 3.

  GripperStatus status;
  status.gripper_activation_action = driver_utils::gACT_lookup().at(response[3] & driver_utils::gACT_mask);
  status.gripper_mode = driver_utils::gMOD_lookup().at(response[3] & driver_utils::gMOD_mask);
  status.gripper_regulate_action = driver_utils::gGTO_lookup().at(response[3] & driver_utils::gGTO_mask);
  status.gripper_activation_status = driver_utils::gSTA_lookup().at(response[3] & driver_utils::gSTA_mask);
  status.object_detection_status = driver_utils::gOBJ_lookup().at(response[3] & driver_utils::gOBJ_mask);
  status.actuator_status = driver_utils::gVAS_lookup().at(response[4] & driver_utils::gVAS_mask);
  status.gripper_fault_status = driver_utils::gFLT_lookup().at(response[5] & driver_utils::gFLT_mask);

  // The requested pressure level in kPa:
  status.max_vacuum_pressure = static_cast<float>(response[6]) - 100;

  // The actual pressure measured kPa:
  status.actual_vacuum_pressure = static_cast<float>(response[7]) - 100;

  return status;
}
}  // namespace epick_driver
