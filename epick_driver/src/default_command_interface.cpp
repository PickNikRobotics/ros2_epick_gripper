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

#include <thread>
#include <chrono>

namespace epick_driver
{
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
  PresetMultipleRegisters = 0x16,
  MasterReadWriteMultipleRegisters = 0x23,
};

DefaultCommandInterface::DefaultCommandInterface(std::unique_ptr<SerialInterface> serial_interface, uint8_t slave_address)
  : serial_interface_{
    std::move(serial_interface),
  }, slave_address_{slave_address}
{
}

bool DefaultCommandInterface::connect()
{
  serial_interface_->open();
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
