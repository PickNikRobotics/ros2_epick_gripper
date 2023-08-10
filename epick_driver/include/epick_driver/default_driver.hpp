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

  /** Connect to the gripper serial connection. */
  bool connect() override;

  /** Disconnect from the gripper serial connection. */
  void disconnect() override;

  /**
   * Set the relative pressure of the gripper in relation to the athmospheric pressure of 100kPa.
   * If the relative pressure is set to -100kPa, the gripper will try to achieve an absolute pressure of 0kPa,
   * which means it will work at full power to hold the object in place.
   * If the relative pressure is set to -22kPa, the gripper will try to achieve the absolute pressure of 78kPa,
   * which means it will hold the object, but not at full power.
   * If the relative pressure is set to 0kPa, the gripper returns to the athmospheric pressure of 100kPa,
   * which will likely cause the object to drop.
   * If the relative pressure is set to anything bigger than 100kPa, the gripper will work in reverse and release the
   * object.
   * @param relative_pressure_kPa The vacuum pressure relative to the athomspheric pressure of 100kPa. Can be anything
   * greater than -100kPa.
   */
  void set_relative_pressure(const float& relative_pressure_kPa) override;

  void set_mode() override;

  void set_max_device_vacuum() override;
  void set_min_device_vacuum() override;

  void set_grip_timeout() override;
  void set_release_time() override;

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
};
}  // namespace epick_driver
