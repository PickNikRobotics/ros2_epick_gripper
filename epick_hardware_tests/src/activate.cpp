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
#include "epick_driver/driver_utils.hpp"

#include "command_line_utility.hpp"

#include <chrono>
#include <memory>
#include <vector>
#include <iostream>

// This is a very basic test to check if the gripper is correctly wired.
// We send an activation request and await for an expected response.

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 500;  // milliseconds
constexpr auto kSlaveAddress = 0x09;

using namespace epick_driver;

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
      "--slave_address", [&slave_address](const char* value) { slave_address = std::stoi(value); }, false);

  GripperMode gripper_mode = GripperMode::AutomaticMode;
  cli.registerHandler(
      "--gripper-mode",
      [&gripper_mode](const char* value) {
        if (strcmp(value, "advanced") == 0)
        {
          gripper_mode = GripperMode::AdvancedMode;
        }
      },
      false);

  double max_vacuum_pressure = -100.0;  // pKa
  cli.registerHandler(
      "--max-vacuum-pressure",
      [&max_vacuum_pressure](const char* value) { max_vacuum_pressure = std::strtod(value, nullptr); }, false);

  double min_vacuum_pressure = -10.0;  // pKa
  cli.registerHandler(
      "--min-vacuum-pressure",
      [&min_vacuum_pressure](const char* value) { min_vacuum_pressure = std::strtod(value, nullptr); }, false);

  std::chrono::milliseconds gripper_timeout = std::chrono::milliseconds(2000);
  cli.registerHandler(
      "--gripper_timeout",
      [&gripper_timeout](const char* value) { gripper_timeout = std::chrono::milliseconds(std::stoi(value)); }, false);

  cli.registerHandler("-h", [&]() {
    std::cout << "Usage: ./set_relative_pressure [OPTIONS]\n"
              << "Options:\n"
              << "  --port VALUE                 Set the com port (default " << kComPort << ")\n"
              << "  --baudrate VALUE             Set the baudrate (default " << kBaudRate << "bps)\n"
              << "  --timeout VALUE              Set the read/write timeout (default " << kTimeout << "ms)\n"
              << "  --slave_address VALUE        Set the slave address (default " << kSlaveAddress << ")\n"
              << "  --max-vacuum-pressure VALUE  Set the max vacuum pressure (default " << max_vacuum_pressure << ")\n"
              << "  --min-vacuum-pressure VALUE  Set the min vacuum pressure (default " << min_vacuum_pressure << ")\n"
              << "  --gripper_timeout VALUE      Set the gipper timeput in millis (default " << gripper_timeout.count()
              << ")\n"
              << "  -h                           Show this help message\n";
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

    auto driver = std::make_unique<DefaultDriver>(std::move(serial), slave_address);
    driver->set_mode(gripper_mode);
    driver->set_max_vacuum_pressure(max_vacuum_pressure);
    driver->set_min_vacuum_pressure(min_vacuum_pressure);
    driver->set_gripper_timeout(gripper_timeout);

    std::cout << "Using the following parameters: " << std::endl;
    std::cout << " - port: " << port << std::endl;
    std::cout << " - baudrate: " << baudrate << "bps" << std::endl;
    std::cout << " - read/write timeut: " << timeout << "ms" << std::endl;
    std::cout << " - slave address: " << slave_address << std::endl;

    std::cout << "Checking if the gripper is connected to /dev/ttyUSB0..." << std::endl;

    bool connected = driver->connect();
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

    epick_driver::GripperStatus status = driver->get_status();

    std::cout << "Status retrieved:" << std::endl;

    std::cout << " - gripper activation action: "
              << driver_utils::gripper_activation_action_to_string(status.gripper_activation_action) << std::endl;
    std::cout << " - gripper regulate action: "
              << driver_utils::gripper_regulate_action_to_string(status.gripper_regulate_action) << std::endl;
    std::cout << " - gripper mode: " << driver_utils::gripper_mode_to_string(status.gripper_mode) << std::endl;
    std::cout << " - object detection status: "
              << driver_utils::object_detection_to_string(status.object_detection_status) << std::endl;
    std::cout << " - gripper fault status: " << driver_utils::fault_status_to_string(status.gripper_fault_status)
              << std::endl;
    std::cout << " - actuator status: " << driver_utils::actuator_status_to_string(status.actuator_status) << std::endl;
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
