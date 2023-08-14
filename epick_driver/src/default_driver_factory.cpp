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

#include "epick_driver/default_driver_factory.hpp"

#include "epick_driver/default_driver.hpp"
#include "epick_driver/default_driver_utils.hpp"
#include "epick_driver/default_serial.hpp"

#include <rclcpp/logging.hpp>

#include <cmath>

namespace epick_driver
{
const auto kLogger = rclcpp::get_logger("DefaultDriverFactory");

std::unique_ptr<epick_driver::Driver>
epick_driver::DefaultDriverFactory::create(const hardware_interface::HardwareInfo& info)
{
  std::string usb_port = info.hardware_parameters.at("usb_port");
  RCLCPP_INFO(kLogger, "usb_port: %s", usb_port.c_str());

  uint8_t slave_address = static_cast<uint8_t>(std::stoul(info.hardware_parameters.at("slave_address")));
  RCLCPP_INFO(kLogger, "slave_address: %d", slave_address);

  uint32_t baud_rate = static_cast<uint32_t>(std::stoul(info.hardware_parameters.at("baud_rate")));
  RCLCPP_INFO(kLogger, "baud_rate: %dbps", baud_rate);

  double timeout_param = std::stod(info.hardware_parameters.at("timeout"));
  uint32_t timeout = static_cast<uint32_t>(round(timeout_param));
  RCLCPP_INFO(kLogger, "timeout: %fms", timeout_param);

  auto serial_interface = std::make_unique<DefaultSerial>();
  serial_interface->set_port(usb_port);
  serial_interface->set_baudrate(baud_rate);
  serial_interface->set_timeout(timeout);

  return std::make_unique<DefaultDriver>(std::move(serial_interface), slave_address);
  ;
}
}  // namespace epick_driver
