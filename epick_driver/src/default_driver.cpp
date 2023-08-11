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

// |-----------------------------+-------------------------------+
// | Gripper Input Registers     | Gripper Output Registers      |
// |-----------------------------+-------------------------------+
// | Address | Function          | Address | Function            |
// |---------+-------------------+---------+---------------------|
// | 0x03E8  | Action Request    | 0x07D0  | Gripper Status      |
// |         | Reserved          |         | Gripper Status Ext. |
// +---------+-------------------+---------+---------------------+
// | 0x03E9  | Reserved          | 0x07D1  | Fault status        |
// |         | Max Rel. Pressure |         | Max Pressure        |
// +---------+-------------------+---------+---------------------+
// | 0x03EA  | Grip Timeout      | 0x07D2  | Actual Pressure     |
// |         | Min Rel. Pressure |         | Reserved            |
// +---------+-------------------+---------+---------------------+
// | 0x03EB  | Reserved          | 0x07D3  | Reserved            |
// |         | Reserved          |         | Reserved            |
// +---------+-------------------+---------+---------------------+
// | 0x03EC  | Reserved          | 0x07D4  | Reserved            |
// |         | Reserved          |         | Reserved            |
// +---------+-------------------+---------+---------------------+
// | 0x03ED  | Reserved          | 0x07D5  | Reserved            |
// |         | Reserved          |         | Reserved            |
// +---------+-------------------+---------+---------------------+
// | 0x03EE  | Reserved          | 0x07D6  | Reserved            |
// |         | Reserved          |         | Reserved            |
// +---------+-------------------+---------+---------------------+
// | 0x03EF  | Reserved          | 0x07D7  | Reserved            |
// |         | Reserved          |         | Reserved            |
// +---------+-------------------+---------+---------------------+

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
  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(driver_utils::FunctionCode::PresetMultipleRegisters),
    data_utils::get_msb(kActionRequestRegisterAddress),
    data_utils::get_lsb(kActionRequestRegisterAddress),
    0x00,  // Number of registers to write MSB.
    0x03,  // Number of registers to write LSB.
    0x06,  // Number of bytes to write.
    0x01,  // Register 1 MSB - set gACT to 1.
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

void DefaultDriver::set_max_vacuum_pressure(const float& vacuum_pressure_kPa)
{
  // Convert the absolute pressure to a value between 0 and 255.
  const uint8_t rPR = static_cast<uint8_t>(std::clamp(std::round(vacuum_pressure_kPa + 100), 0.0f, 255.0f));

  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(driver_utils::FunctionCode::PresetSingleRegister),
    0x03,  // Register address MSB
    0xE9,  // Register address LSB
    0x00,  // Reserved byte
    rPR    // The absolute pressure.
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
}

void DefaultDriver::set_min_vacuum_pressure(const float& vacuum_pressure_kPa)
{
  // Convert the absolute pressure to a value between 0 and 255.
  const uint8_t rPR = static_cast<uint8_t>(std::clamp(std::round(vacuum_pressure_kPa + 100), 0.0f, 255.0f));

  std::vector<uint8_t> request = {
    slave_address_,
    static_cast<uint8_t>(driver_utils::FunctionCode::PresetSingleRegister),
    0x03,  // Register address MSB
    0xEA,  // Register address LSB
    0x00,  // Reserved byte
    rPR    // The absolute pressure.
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
}

void DefaultDriver::set_mode()
{
}

void DefaultDriver::set_max_device_vacuum()
{
}

void DefaultDriver::set_min_device_vacuum()
{
}

void DefaultDriver::set_grip_timeout()
{
}

void DefaultDriver::set_release_time()
{
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
