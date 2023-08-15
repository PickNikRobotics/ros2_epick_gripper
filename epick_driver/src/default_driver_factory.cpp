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

constexpr auto kUsbPortParamName = "usb_port";
constexpr auto kUsbPortParamDefault = "/dev/ttyUSB0";

constexpr auto kSlaveAddressParamName = "slave_address";
constexpr auto kSlaveAddressParamDefault = 0x9;

constexpr auto kBaudrateParamName = "baudrate";
constexpr auto kBaudrateAddressParamDefault = 115200;

constexpr auto kTimeoutParamName = "timeout";
constexpr auto kTimeoutParamDefault = 500;

constexpr auto kModeParamName = "mode";
constexpr auto kModeParamDefault = GripperMode::AutomaticMode;

constexpr auto kMaxVacuumPressureParamName = "max_vacuum_pressure";
constexpr auto kMaxVacuumPressureParamDefault = -100.0;  // kPA

constexpr auto kMinVacuumPressureParamName = "min_vacuum_pressure";
constexpr auto kMinVacuumPressureParamDefault = 0.0;  // kPA

constexpr auto kGripperTimeoutParamName = "gripper_timeout";
constexpr auto kGripperTimeoutParamDefault = 500;  // ms

const auto kLogger = rclcpp::get_logger("DefaultDriverFactory");

std::unique_ptr<epick_driver::Driver>
epick_driver::DefaultDriverFactory::create(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(kLogger, "Reading usb_port...");
  std::string usb_port = info.hardware_parameters.count(kUsbPortParamName) ?
                             info.hardware_parameters.at(kUsbPortParamName) :
                             kUsbPortParamDefault;
  RCLCPP_INFO(kLogger, "usb_port: %s", usb_port.c_str());

  RCLCPP_INFO(kLogger, "Reading slave_address...");
  uint8_t slave_address = info.hardware_parameters.count(kSlaveAddressParamName) ?
                              static_cast<uint8_t>(std::stoul(info.hardware_parameters.at(kSlaveAddressParamName))) :
                              kSlaveAddressParamDefault;
  RCLCPP_INFO(kLogger, "slave_address: %d", slave_address);

  RCLCPP_INFO(kLogger, "Reading baudrate...");
  uint32_t baudrate = info.hardware_parameters.count(kBaudrateParamName) ?
                          static_cast<uint32_t>(std::stoul(info.hardware_parameters.at(kBaudrateParamName))) :
                          kBaudrateAddressParamDefault;
  RCLCPP_INFO(kLogger, "baudrate: %dbps", baudrate);

  RCLCPP_INFO(kLogger, "Reading timeout...");
  uint32_t timeout = static_cast<uint32_t>(info.hardware_parameters.count(kTimeoutParamName) ?
                                               std::stoul(info.hardware_parameters.at(kTimeoutParamName)) :
                                               kTimeoutParamDefault);
  RCLCPP_INFO(kLogger, "timeout: %dms", timeout);

  RCLCPP_INFO(kLogger, "Reading mode...");
  GripperMode mode = info.hardware_parameters.count(kModeParamName) ?
                         info.hardware_parameters.at(kModeParamName) ==
                                 default_driver_utils::gripper_mode_to_string(GripperMode::AdvancedMode) ?
                         GripperMode::AdvancedMode :
                         kModeParamDefault :
                         kModeParamDefault;
  RCLCPP_INFO(kLogger, "mode: %s", default_driver_utils::gripper_mode_to_string(mode).c_str());

  RCLCPP_INFO(kLogger, "Reading max vacuum pressure...");
  double max_vacuum_pressure = info.hardware_parameters.count(kMaxVacuumPressureParamName) ?
                                   std::stod(info.hardware_parameters.at(kMaxVacuumPressureParamName)) :
                                   kMaxVacuumPressureParamDefault;
  RCLCPP_INFO(kLogger, "%s: %fkPa", kMaxVacuumPressureParamName, max_vacuum_pressure);

  RCLCPP_INFO(kLogger, "Reading min vacuum pressure...");
  double min_vacuum_pressure = info.hardware_parameters.count(kMinVacuumPressureParamName) ?
                                   std::stod(info.hardware_parameters.at(kMinVacuumPressureParamName)) :
                                   kMinVacuumPressureParamDefault;
  RCLCPP_INFO(kLogger, "%s: %fkPa", kMinVacuumPressureParamName, min_vacuum_pressure);

  RCLCPP_INFO(kLogger, "Reading gripper timeout...");
  std::chrono::milliseconds gripper_timeout =
      info.hardware_parameters.count(kGripperTimeoutParamName) ?
          std::chrono::milliseconds(std::stoi(info.hardware_parameters.at(kGripperTimeoutParamName))) :
          std::chrono::milliseconds(kGripperTimeoutParamDefault);
  RCLCPP_INFO(kLogger, "%s: %ldms", kGripperTimeoutParamName, gripper_timeout.count());

  auto serial = std::make_unique<DefaultSerial>();
  serial->set_port(usb_port);
  serial->set_baudrate(baudrate);
  serial->set_timeout(timeout);

  auto driver = std::make_unique<DefaultDriver>(std::move(serial), slave_address);
  driver->set_mode(mode);
  driver->set_max_vacuum_pressure(max_vacuum_pressure);
  driver->set_min_vacuum_pressure(min_vacuum_pressure);
  driver->set_gripper_timeout(gripper_timeout);

  return driver;
}
}  // namespace epick_driver
