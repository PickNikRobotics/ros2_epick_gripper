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

#include "epick_driver/default_command_interface_factory.hpp"

#include "epick_driver/default_command_interface.hpp"
#include "epick_driver/default_serial_interface.hpp"

#include <rclcpp/logging.hpp>

#include <cmath>

namespace epick_driver
{
const auto kLogger = rclcpp::get_logger("DefaultCommandInterfaceFactory");

std::unique_ptr<epick_driver::CommandInterface>
epick_driver::DefaultCommandInterfaceFactory::create(const hardware_interface::HardwareInfo& info)
{
  std::string usb_port = info.hardware_parameters.at("connection.usb_port");
  RCLCPP_INFO(kLogger, "connection.usb_port: %s", usb_port.c_str());

  uint8_t slave_address = static_cast<uint8_t>(std::stoul(info.hardware_parameters.at("connection.slave_address")));
  RCLCPP_INFO(kLogger, "connection.slave_address: %d", slave_address);

  uint32_t baud_rate = static_cast<uint32_t>(std::stoul(info.hardware_parameters.at("connection.baud_rate")));
  RCLCPP_INFO(kLogger, "connection.baud_rate: %d", baud_rate);

  double timeout = std::stod(info.hardware_parameters.at("connection.timeout"));
  uint32_t timeout_ms = static_cast<uint32_t>(round(timeout * 1e3));
  RCLCPP_INFO(kLogger, "connection.timeout: %f", timeout);

  double grasp_max_relative_pressure = std::stod(info.hardware_parameters.at("grip.max_relative_pressure"));
  RCLCPP_INFO(kLogger, "grip.max_relative_pressure: %f", grasp_max_relative_pressure);

  double grasp_min_relative_pressure = std::stod(info.hardware_parameters.at("grip.min_relative_pressure"));
  RCLCPP_INFO(kLogger, "grip.min_relative_pressure: %f", grasp_min_relative_pressure);

  double grasp_action_timeout = std::stod(info.hardware_parameters.at("grip.action_timeout"));
  RCLCPP_INFO(kLogger, "grip.action_timeout: %f", grasp_action_timeout);

  double drop_max_relative_pressure = std::stod(info.hardware_parameters.at("release.max_relative_pressure"));
  RCLCPP_INFO(kLogger, "release.max_prelative_ressure: %f", drop_max_relative_pressure);

  double drop_min_relative_pressure = std::stod(info.hardware_parameters.at("release.min_relative_pressure"));
  RCLCPP_INFO(kLogger, "release.min_relative_pressure: %f", drop_min_relative_pressure);

  double drop_action_timeout = std::stod(info.hardware_parameters.at("release.action_timeout"));
  RCLCPP_INFO(kLogger, "release.action_timeout: %f", drop_action_timeout);

  auto serial_interface = std::make_unique<DefaultSerialInterface>();
  serial_interface->set_port(usb_port);
  serial_interface->set_baudrate(baud_rate);
  serial_interface->set_timeout(timeout_ms);

  auto command_interface = std::make_unique<DefaultCommandInterface>(std::move(serial_interface), slave_address);
  command_interface->set_mode();
  command_interface->set_release_time();
  // TODO: set all relevant parameters.

  return command_interface;
}
}  // namespace epick_driver
