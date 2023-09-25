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

#include <cmath>

#include <epick_driver/default_driver_factory.hpp>

#include <epick_driver/data_utils.hpp>
#include <epick_driver/default_driver.hpp>
#include <epick_driver/default_driver_utils.hpp>
#include <epick_driver/fake/fake_driver.hpp>
#include <epick_driver/default_serial.hpp>
#include <epick_driver/default_serial_factory.hpp>

#include <rclcpp/logging.hpp>

namespace epick_driver
{

const auto kLogger = rclcpp::get_logger("DefaultDriverFactory");

constexpr auto kSlaveAddressParamName = "slave_address";
constexpr uint8_t kSlaveAddressParamDefault = 0x9;

constexpr auto kModeParamName = "mode";
constexpr auto kModeParamDefault = GripperMode::AutomaticMode;

constexpr auto kGripMaxVacuumPressureParamName = "grip_max_vacuum_pressure";
constexpr auto kGripMaxVacuumPressureParamDefault = -100.0;  // kPA

constexpr auto kGripMinVacuumPressureParamName = "grip_min_vacuum_pressure";
constexpr auto kGripMinVacuumPressureParamDefault = -10.0;  // kPA

constexpr auto kGripTimeoutParamName = "grip_timeout";
constexpr auto kGripTimeoutParamDefault = 0.5;

constexpr auto kReleaseTimeoutParamName = "release_timeout";
constexpr auto kReleaseTimeoutParamDefault = 0.5;

constexpr auto kUseDummyParamName = "use_dummy";
constexpr auto kUseDummyParamDefault = "False";  // TODO(kineticsystem): make this so it's not case sensitive

using data_utils::to_lower;

std::unique_ptr<epick_driver::Driver>
epick_driver::DefaultDriverFactory::create(const hardware_interface::HardwareInfo& info) const
{
  RCLCPP_INFO(kLogger, "Reading slave_address...");
  // Convert base-16 address stored as a string (for example, "0x9") into an integer
  const uint8_t slave_address =
      info.hardware_parameters.count(kSlaveAddressParamName) ?
          static_cast<uint8_t>(std::stoul(info.hardware_parameters.at(kSlaveAddressParamName), nullptr, 16)) :
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

  RCLCPP_INFO(kLogger, "Reading grip max vacuum pressure...");
  double grip_max_vacuum_pressure = info.hardware_parameters.count(kGripMaxVacuumPressureParamName) ?
                                        std::stod(info.hardware_parameters.at(kGripMaxVacuumPressureParamName)) :
                                        kGripMaxVacuumPressureParamDefault;
  RCLCPP_INFO(kLogger, "%s: %fkPa", kGripMaxVacuumPressureParamName, grip_max_vacuum_pressure);

  RCLCPP_INFO(kLogger, "Reading grip min vacuum pressure...");
  double grip_min_vacuum_pressure = info.hardware_parameters.count(kGripMinVacuumPressureParamName) ?
                                        std::stod(info.hardware_parameters.at(kGripMinVacuumPressureParamName)) :
                                        kGripMinVacuumPressureParamDefault;
  RCLCPP_INFO(kLogger, "%s: %fkPa", kGripMinVacuumPressureParamName, grip_min_vacuum_pressure);

  RCLCPP_INFO(kLogger, "Reading grip timeout...");
  double grip_timeout = info.hardware_parameters.count(kGripTimeoutParamName) ?
                            std::stod(info.hardware_parameters.at(kGripTimeoutParamName)) :
                            kGripTimeoutParamDefault;
  RCLCPP_INFO(kLogger, "%s: %fs", kGripTimeoutParamName, grip_timeout);

  RCLCPP_INFO(kLogger, "Reading release timeout...");
  double release_timeout = info.hardware_parameters.count(kReleaseTimeoutParamName) ?
                               std::stod(info.hardware_parameters.at(kReleaseTimeoutParamName)) :
                               kReleaseTimeoutParamDefault;
  RCLCPP_INFO(kLogger, "%s: %fs", kReleaseTimeoutParamName, release_timeout);

  auto driver = create_driver(info);
  driver->set_slave_address(slave_address);
  driver->set_mode(mode);
  driver->set_grip_max_vacuum_pressure(grip_max_vacuum_pressure);
  driver->set_grip_min_vacuum_pressure(grip_min_vacuum_pressure);
  driver->set_grip_timeout(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(grip_timeout)));
  driver->set_release_timeout(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(release_timeout)));
  return driver;
}

std::unique_ptr<Driver> DefaultDriverFactory::create_driver(const hardware_interface::HardwareInfo& info) const
{
  // We give the user an option to startup a dummy gripper for testing purposes.
  if (info.hardware_parameters.count(kUseDummyParamName) &&
      to_lower(info.hardware_parameters.at(kUseDummyParamName)) != to_lower(kUseDummyParamDefault))
  {
    RCLCPP_INFO(kLogger, "You are connected to a dummy driver, not a real hardware.");
    return std::make_unique<FakeDriver>();
  }
  else
  {
    auto serial = DefaultSerialFactory().create(info);
    return std::make_unique<DefaultDriver>(std::move(serial));
  }
}
}  // namespace epick_driver
