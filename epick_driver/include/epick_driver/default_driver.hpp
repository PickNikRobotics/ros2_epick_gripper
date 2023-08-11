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

#include "epick_driver/driver.hpp"
#include "epick_driver/serial.hpp"

#include <chrono>
#include <functional>
#include <memory>

namespace epick_driver
{
class DefaultDriver : public Driver
{
public:
  /**
   * Initialize the interface to control the Robotiq EPick gripper.
   * @param serial_interface The serial connection.
   * @param slave_address The slave address as specified in the MODBUS RTU protocol.
   */
  explicit DefaultDriver(std::unique_ptr<Serial> serial_interface, uint8_t slave_address);

  bool connect() override;
  void disconnect() override;

  void set_mode(const GripperMode gripper_mode) override;
  void set_max_vacuum_pressure(const float& vacuum_pressure_kPa) override;
  void set_min_vacuum_pressure(const float& vacuum_pressure_kPa) override;
  void set_gripper_timeout(std::chrono::milliseconds gripper_timeout) override;

  GripperStatus get_status() override;

  /** Activate the gripper with the specified operation mode and parameters. */
  void activate() override;

  /** Deactivate the gripper. */
  void deactivate() override;

  void grip() override;

  void release() override;

private:
  std::unique_ptr<Serial> serial_interface_;
  uint8_t slave_address_;

  GripperMode gripper_mode_ = GripperMode::AutomaticMode;
  float max_vacuum_pressure_ = -100;
  float min_vacuum_pressure_ = -10;
  std::chrono::milliseconds gripper_timeout_;
};
}  // namespace epick_driver
