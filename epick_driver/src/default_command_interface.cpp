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

#include "epick_driver/default_command_interface.hpp"

#include "epick_driver/crc_utils.hpp"
#include "epick_driver/data_utils.hpp"

#include "serial/serial.h"

#include <rclcpp/logging.hpp>

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

const auto kLogger = rclcpp::get_logger("DefaultCommandInterface");

enum class kFunctionCode : uint8_t
{
  ReadInputRegisters = 0x04,
  PresetSingleRegister = 0x06,
  PresetMultipleRegisters = 0x10,
  MasterReadWriteMultipleRegisters = 0x17,
};

constexpr uint8_t gACT_mask = 0b00000001;  // Activate echo
constexpr uint8_t gMOD_mask = 0b00000110;  // Gripper mode echo
constexpr uint8_t gGTO_mask = 0b00001000;  // Regulate echo
constexpr uint8_t gSTA_mask = 0b00110000;  // Activation status
constexpr uint8_t gOBJ_mask = 0b11000000;  // Object status
constexpr uint8_t gVAS_mask = 0b00000011;  // Vacuum actuator status
constexpr uint8_t kFLT_mask = 0b11110000;  //
constexpr uint8_t gFLT_mask = 0b00001111;  // Gripper fault status

enum class GripperActivation
{
  Inactive,  // 0b0
  Active,    // 0b1
};

enum class ObjectDetection
{
  Unknown,
  ObjectDetected,
  NoObjectDetected
};

enum class Regulate
{

};

enum class GripperMode
{
  AutomaticMode,
  AdvancedMode,
  Reserved
};

struct GripperStatus
{
  GripperActivation activation;
  GripperMode mode;
  ObjectDetection object_detection;
};

GripperStatus generateStatus(uint8_t register_value)
{
  GripperStatus status;

  uint8_t gACT = register_value & gACT_mask;
  switch (gACT)
  {
    case 0b0:
      status.activation = GripperActivation::Inactive;
      break;
    case 0b1:
      status.activation = GripperActivation::Active;
      break;
  }

  uint8_t gMOD = (register_value & gMOD_mask) >> 1;
  switch (gMOD)
  {
    case 0b00:
      status.mode = GripperMode::AutomaticMode;
      break;
    case 0b01:
      status.mode = GripperMode::AdvancedMode;
      break;
    case 0b10:
      status.mode = GripperMode::Reserved;
      break;
    case 0b11:
      status.mode = GripperMode::Reserved;
      break;
  }

  uint8_t gGTO = (register_value & gGTO_mask) >> 3;
  switch (gGTO)
  {
    case 0b0:
      status.activation = GripperActivation::Inactive;
      break;
    case 0b1:
      status.activation = GripperActivation::Active;
      break;
  }

  uint8_t gOBJ = (register_value & gOBJ_mask) >> 4;
  switch (gOBJ)
  {
    case 0b00:
      status.object_detection = ObjectDetection::Unknown;
      break;
    case 0b01:
      status.object_detection = ObjectDetection::ObjectDetected;
      break;
    case 0b10:
      status.object_detection = ObjectDetection::ObjectDetected;
      break;
    case 0b11:
      status.object_detection = ObjectDetection::NoObjectDetected;
      break;
  }

  return status;
}

DefaultCommandInterface::DefaultCommandInterface(std::unique_ptr<SerialInterface> serial_interface, uint8_t slave_address)
  : serial_interface_{
    std::move(serial_interface),
  }, slave_address_{slave_address}
{
}

bool DefaultCommandInterface::connect()
{
  serial_interface_->open();

  // TODO: ask for the gripper status. If the gripper answer, then all is up and running.

  return serial_interface_->is_open();
}

void DefaultCommandInterface::disconnect()
{
  serial_interface_->close();
}

void DefaultCommandInterface::activate()
{
  // Set rACT to 1, clear all other registers.
  const auto request = createCommand(slave_address_, static_cast<uint8_t>(kFunctionCode::PresetMultipleRegisters),
                                     kActionRequestRegisterAddress, { 0x0100, 0x0000, 0x0000 });
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

void DefaultCommandInterface::deactivate()
{
  // Clear all registers.
  const auto request = createCommand(slave_address_, static_cast<uint8_t>(kFunctionCode::PresetMultipleRegisters),
                                     kActionRequestRegisterAddress, { 0x0000, 0x0000, 0x0000 });
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

void DefaultCommandInterface::set_mode()
{
}

void DefaultCommandInterface::set_max_device_vacuum()
{
}

void DefaultCommandInterface::set_min_device_vacuum()
{
}

void DefaultCommandInterface::set_grip_timeout()
{
}

void DefaultCommandInterface::set_release_time()
{
}

void DefaultCommandInterface::get_status()
{
  constexpr uint16_t num_registers_to_read = 0x0003;
  std::vector<uint8_t> request = { slave_address_,
                                   static_cast<uint8_t>(kFunctionCode::ReadInputRegisters),
                                   data_utils::get_msb(kGripperStatusRegister),
                                   data_utils::get_lsb(kGripperStatusRegister),
                                   data_utils::get_msb(num_registers_to_read),
                                   data_utils::get_lsb(num_registers_to_read) };
  auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));

  try
  {
    serial_interface_->write(request);
    constexpr uint16_t response_size = 0x000B;
    auto response = serial_interface_->read(response_size);

    std::cout << data_utils::to_hex(response) << std::endl;
  }
  catch (const serial::IOException& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to read the gripper status: %s", e.what());
    throw;
  }
}

std::vector<uint8_t> DefaultCommandInterface::createCommand(uint8_t slave_address, uint8_t function_code,
                                                            uint16_t first_register_address,
                                                            const std::vector<uint16_t>& data)
{
  uint16_t num_registers = data.size();
  uint8_t num_bytes = 2 * num_registers;

  std::vector<uint8_t> cmd = { slave_address,
                               function_code,
                               data_utils::get_msb(first_register_address),
                               data_utils::get_lsb(first_register_address),
                               data_utils::get_msb(num_registers),
                               data_utils::get_lsb(num_registers),
                               num_bytes };
  for (auto byte : data)
  {
    cmd.push_back(data_utils::get_msb(byte));
    cmd.push_back(data_utils::get_lsb(byte));
  }

  auto crc = crc_utils::compute_crc(cmd);
  cmd.push_back(data_utils::get_msb(crc));
  cmd.push_back(data_utils::get_lsb(crc));

  return cmd;
}
}  // namespace epick_driver
