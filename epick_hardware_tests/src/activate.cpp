// Copyright (c) 2022 PickNik, Inc.
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

#include "epick_driver/default_driver.hpp"
#include "epick_driver/default_serial.hpp"
#include "epick_driver/default_driver_utils.hpp"

#include "command_line_utility.hpp"

#include <chrono>
#include <memory>
#include <vector>
#include <iostream>

// This is a command to activate the gripper and read its status.

using namespace epick_driver;

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 500;  // milliseconds
constexpr auto kSlaveAddress = 0x09;
constexpr auto kGripperMode = GripperMode::AutomaticMode;
constexpr auto kGripMaxVacuumPressure = -100.0f;  // kPa
constexpr auto kGripMinVacuumPressure = -10.0f;   // kPa
constexpr auto kReleaseVacuumPressure = 50.0f;    // kPa
constexpr auto kGripperTimeout = std::chrono::milliseconds(2000);

int main(int argc, char* argv[])
{
  CommandLineUtility cli;

  std::string port = kComPort;
  cli.registerHandler(
      "--port", [&port](const char* value) { port = value; }, false);

  int baudrate = kBaudRate;
  cli.registerHandler(
      "--baudrate", [&baudrate](const char* value) { baudrate = std::stoi(value); }, false);

  int timeout = kTimeout;
  cli.registerHandler(
      "--timeout", [&timeout](const char* value) { timeout = std::stoi(value); }, false);

  int slave_address = kSlaveAddress;
  cli.registerHandler(
      "--slave-address", [&slave_address](const char* value) { slave_address = std::stoi(value); }, false);

  GripperMode gripper_mode = kGripperMode;
  cli.registerHandler(
      "--gripper-mode",
      [&gripper_mode](const char* value) {
        if (strcmp(value, "AdvancedMode") == 0)
        {
          gripper_mode = GripperMode::AdvancedMode;
        }
      },
      false);

  float grip_max_vacuum_pressure = kGripMaxVacuumPressure;  // pKa
  cli.registerHandler(
      "--grip-max-vacuum-pressure",
      [&grip_max_vacuum_pressure](const char* value) { grip_max_vacuum_pressure = std::strtof(value, nullptr); },
      false);

  float grip_min_vacuum_pressure = kGripMinVacuumPressure;  // pKa
  cli.registerHandler(
      "--min-vacuum-pressure",
      [&grip_min_vacuum_pressure](const char* value) { grip_min_vacuum_pressure = std::strtof(value, nullptr); },
      false);

  float release_vacuum_pressure = kReleaseVacuumPressure;  // pKa
  cli.registerHandler(
      "--release-vacuum-pressure",
      [&release_vacuum_pressure](const char* value) { release_vacuum_pressure = std::strtof(value, nullptr); }, false);

  std::chrono::milliseconds gripper_timeout = kGripperTimeout;
  cli.registerHandler(
      "--gripper-timeout",
      [&gripper_timeout](const char* value) { gripper_timeout = std::chrono::milliseconds(std::stoi(value)); }, false);

  cli.registerHandler("-h", [&]() {
    std::cout << "Usage: ./set_relative_pressure [OPTIONS]\n"
              << "Options:\n"
              << "  --port VALUE                      Set the com port (default " << kComPort << ")\n"
              << "  --baudrate VALUE                  Set the baudrate (default " << kBaudRate << "bps)\n"
              << "  --timeout VALUE                   Set the read/write timeout (default " << kTimeout << "ms)\n"
              << "  --slave-address VALUE             Set the slave address (default " << kSlaveAddress << ")\n"
              << "  --gripper-mode VALUE              Set the gripper mode (default "
              << default_driver_utils::gripper_mode_to_string(kGripperMode) << ")\n"
              << "                                    Valid values AutomaticMode, AdvancedMode\n"
              << default_driver_utils::gripper_mode_to_string(kGripperMode) << ")\n"
              << "  --grip-max-vacuum-pressure VALUE  Set the max vacuum pressure (default " << kGripMaxVacuumPressure
              << ")\n"
              << "  --grip-min-vacuum-pressure VALUE  Set the min vacuum pressure (default " << kGripMinVacuumPressure
              << ")\n"
              << "  --release-vacuum-pressure VALUE   Set the min vacuum pressure (default " << kGripMinVacuumPressure
              << ")\n"
              << "  --gripper-timeout VALUE           Set the gripper timeput in millis (default "
              << kGripperTimeout.count() << ")\n"
              << "  -h                                Show this help message\n";
    exit(0);
  });

  if (!cli.parse(argc, argv))
  {
    return 1;
  }

  try
  {
    auto serial = std::make_unique<DefaultSerial>();
    serial->set_port(port);
    serial->set_baudrate(baudrate);
    serial->set_timeout(timeout);

    auto driver = std::make_unique<DefaultDriver>(std::move(serial));
    driver->set_slave_address(slave_address);
    driver->set_mode(gripper_mode);
    driver->set_grip_max_vacuum_pressure(grip_max_vacuum_pressure);
    driver->set_grip_min_vacuum_pressure(grip_min_vacuum_pressure);
    driver->set_release_vacuum_pressure(release_vacuum_pressure);
    driver->set_grip_timeout(gripper_timeout);

    std::cout << "Using the following parameters: " << std::endl;
    std::cout << " - port: " << port << std::endl;
    std::cout << " - baudrate: " << baudrate << "bps" << std::endl;
    std::cout << " - read/write timeut: " << timeout << "ms" << std::endl;
    std::cout << " - slave address: " << slave_address << std::endl;
    std::cout << " - gripper mode: " << default_driver_utils::gripper_mode_to_string(gripper_mode) << std::endl;
    std::cout << " - grip max vacuum pressure: " << grip_max_vacuum_pressure << "kPa" << std::endl;
    std::cout << " - grip min vacuum pressure: " << grip_min_vacuum_pressure << "kPa" << std::endl;
    std::cout << " - release vacuum pressure: " << release_vacuum_pressure << "kPa" << std::endl;
    std::cout << " - gripper timeout: " << gripper_timeout.count() << "ms" << std::endl;

    std::cout << "Checking if the gripper is connected..." << std::endl;

    const bool connected = driver->connect();
    if (!connected)
    {
      std::cout << "The gripper is not connected" << std::endl;
      return 1;
    }

    std::cout << "The gripper is connected." << std::endl;
    std::cout << "Activating the gripper..." << std::endl;

    driver->activate();

    std::cout << "The gripper is activated." << std::endl;

    std::cout << "Reading the gripper status..." << std::endl;

    const auto status = driver->get_status();

    std::cout << "Status retrieved:" << std::endl;

    std::cout << " - gripper activation action: "
              << default_driver_utils::gripper_activation_action_to_string(status.gripper_activation_action)
              << std::endl;
    std::cout << " - gripper regulate action: "
              << default_driver_utils::gripper_regulate_action_to_string(status.gripper_regulate_action) << std::endl;
    std::cout << " - gripper mode: " << default_driver_utils::gripper_mode_to_string(status.gripper_mode) << std::endl;
    std::cout << " - object detection status: "
              << default_driver_utils::object_detection_to_string(status.object_detection_status) << std::endl;
    std::cout << " - gripper fault status: "
              << default_driver_utils::fault_status_to_string(status.gripper_fault_status) << std::endl;
    std::cout << " - actuator status: " << default_driver_utils::actuator_status_to_string(status.actuator_status)
              << std::endl;
    std::cout << " - max vacuum pressure: " << status.max_vacuum_pressure << "kPa" << std::endl;
    std::cout << " - actual vacuum pressure: " << status.actual_vacuum_pressure << "kPa" << std::endl;
  }
  catch (const serial::IOException& e)
  {
    std::cout << "Failed to communicating with the gripper:" << e.what();
    return 1;
  }

  return 0;
}
