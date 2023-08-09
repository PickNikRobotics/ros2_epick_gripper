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

#include "epick_driver/command_interface.hpp"
#include "epick_driver/serial_interface.hpp"

#include <functional>
#include <memory>

namespace epick_driver
{

class DefaultCommandInterface : public CommandInterface
{
public:
  /**
   * Initialize the interface to control the Robotiq EPick gripper.
   * @param serial_interface The serial connection.
   * @param slave_address The slave address as specified in the MODBUS RTU protocol.
   */
  explicit DefaultCommandInterface(std::unique_ptr<SerialInterface> serial_interface, uint8_t slave_address);

  /** Connect to the gripper serial connection. */
  bool connect() override;

  /** Disconnect from the gripper serial connection. */
  void disconnect() override;

  void set_mode() override;

  void set_max_device_vacuum() override;
  void set_min_device_vacuum() override;

  void set_grip_timeout() override;
  void set_release_time() override;

  void get_status() override;

  /** Activate the gripper with the specified operation mode and parameters. */
  void activate() override;

  /** Deactivate the gripper. */
  void deactivate() override;

private:
  std::unique_ptr<SerialInterface> serial_interface_;
  uint8_t slave_address_;

  std::vector<uint8_t> createCommand(uint8_t slave_address, uint8_t function_code, uint16_t first_register_address,
                                     const std::vector<uint16_t>& data);
};
}  // namespace epick_driver
