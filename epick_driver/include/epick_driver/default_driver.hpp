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

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <epick_driver/driver.hpp>
#include <epick_driver/serial.hpp>

namespace epick_driver
{
/**
 * This is the default implementation of the Driver to control the Epick Gripper.
 */
class DefaultDriver : public Driver
{
public:
  /**
   * Initialize the interface to control the Robotiq EPick gripper.
   * @param serial_interface The serial connection.
   * @param slave_address The slave address as specified in the MODBUS RTU protocol.
   */
  explicit DefaultDriver(std::unique_ptr<Serial> serial);

  bool connect() override;
  void disconnect() override;

  void set_slave_address(uint8_t slave_address) override;
  void set_mode(GripperMode gripper_mode) override;
  void set_grip_max_vacuum_pressure(float vacuum_pressure) override;
  void set_grip_min_vacuum_pressure(float vacuum_pressure) override;
  void set_grip_timeout(std::chrono::milliseconds grip_timeout) override;
  void set_release_timeout(std::chrono::milliseconds release_timeout) override;

  GripperStatus get_status() override;

  /** Activate the gripper with the specified operation mode and parameters. */
  void activate() override;

  /** Deactivate the gripper. */
  void deactivate() override;

  void grip() override;

  void release() override;

private:
  /**
   * With this command we send a request and wait for a response of given size.
   * Behind the scene, if the response is not received, the software makes an attempt
   * to resend the command up to 5 times before returning an empty response.
   * @param request The command request.
   * @param response_size The response expected size.
   * @return The response or an empty vector if an en error occurred.
   */
  std::vector<uint8_t> send(const std::vector<uint8_t>& request, size_t response_size) const;

  std::unique_ptr<Serial> serial_ = nullptr;
  uint8_t slave_address_ = 0x00;
  GripperMode gripper_mode_ = GripperMode::AutomaticMode;

  float grip_max_vacuum_pressure_ = 0.0;
  float grip_min_vacuum_pressure_ = 0.0;

  std::chrono::milliseconds grip_timeout_ = std::chrono::milliseconds(0);
  std::chrono::milliseconds release_timeout_ = std::chrono::milliseconds(0);
};
}  // namespace epick_driver
