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

#include <epick_driver/default_driver_factory.hpp>

#include <epick_driver/default_driver.hpp>
#include <epick_driver/default_driver_utils.hpp>
#include <epick_driver/default_serial.hpp>
#include <epick_driver/default_serial_factory.hpp>
#include <epick_driver/fake/fake_driver.hpp>

#include <rclcpp/logging.hpp>

#include <cmath>

namespace epick_driver
{

const auto kLogger = rclcpp::get_logger("DefaultDriverFactory");

constexpr auto kSlaveAddressParamName = "slave_address";
constexpr auto kSlaveAddressParamDefault = 0x9;

constexpr auto kModeParamName = "mode";
constexpr auto kModeParamDefault = GripperMode::AutomaticMode;

constexpr auto kMaxVacuumPressureParamName = "max_vacuum_pressure";
constexpr auto kMaxVacuumPressureParamDefault = -100.0;  // kPA

constexpr auto kMinVacuumPressureParamName = "min_vacuum_pressure";
constexpr auto kMinVacuumPressureParamDefault = 0.0;  // kPA

constexpr auto kGripperTimeoutParamName = "gripper_timeout";
constexpr auto kGripperTimeoutParamDefault = 500;  // ms

constexpr auto kUseDummyParamName = "use_dummy";
constexpr auto kUseDummyParamDefault = "false";

std::unique_ptr<epick_driver::Driver>
epick_driver::DefaultDriverFactory::create(const hardware_interface::HardwareInfo& info) const
{
  RCLCPP_INFO(kLogger, "Reading slave_address...");
  uint8_t slave_address = info.hardware_parameters.count(kSlaveAddressParamName) ?
                              static_cast<uint8_t>(std::stoul(info.hardware_parameters.at(kSlaveAddressParamName))) :
                              kSlaveAddressParamDefault;
  RCLCPP_INFO(kLogger, "slave_address: %d", slave_address);

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

  auto driver = create_driver(info);
  driver->set_slave_address(slave_address);
  driver->set_mode(mode);
  driver->set_max_vacuum_pressure(max_vacuum_pressure);
  driver->set_min_vacuum_pressure(min_vacuum_pressure);
  driver->set_gripper_timeout(gripper_timeout);

  return driver;
}

std::unique_ptr<Driver> DefaultDriverFactory::create_driver(const hardware_interface::HardwareInfo& info) const
{
  auto serial = DefaultSerialFactory().create(info);
  return std::make_unique<DefaultDriver>(std::move(serial));
}
}  // namespace epick_driver
