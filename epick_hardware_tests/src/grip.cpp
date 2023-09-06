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

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include <epick_driver/default_driver.hpp>
#include <epick_driver/default_serial.hpp>
#include <epick_driver/default_driver_utils.hpp>

#include "command_line_utility.hpp"

// Grip the object.

using epick_driver::DefaultDriver;
using epick_driver::DefaultSerial;
using epick_driver::GripperMode;
using epick_driver::default_driver_utils::actuator_status_to_string;
using epick_driver::default_driver_utils::fault_status_to_string;
using epick_driver::default_driver_utils::gripper_activation_action_to_string;
using epick_driver::default_driver_utils::gripper_mode_to_string;
using epick_driver::default_driver_utils::gripper_regulate_action_to_string;
using epick_driver::default_driver_utils::object_detection_to_string;

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 0.5;
constexpr auto kSlaveAddress = 0x09;

int main(int argc, char* argv[])
{
  CommandLineUtility cli;

  std::string port = kComPort;
  cli.registerHandler(
      "--port", [&port](const char* value) { port = value; }, false);

  int baudrate = kBaudRate;
  cli.registerHandler(
      "--baudrate", [&baudrate](const char* value) { baudrate = std::stoi(value); }, false);

  double timeout = kTimeout;

  cli.registerHandler(
      "--timeout", [&timeout](const char* value) { timeout = std::stod(value); }, false);

  int slave_address = kSlaveAddress;
  cli.registerHandler(
      "--slave-address", [&slave_address](const char* value) { slave_address = std::stoi(value); }, false);

  cli.registerHandler("-h", [&]() {
    std::cout << "Usage: ./set_relative_pressure [OPTIONS]\n"
              << "Options:\n"
              << "  --port VALUE                 Set the com port (default " << kComPort << ")\n"
              << "  --baudrate VALUE             Set the baudrate (default " << kBaudRate << "bps)\n"
              << "  --timeout VALUE              Set the read/write timeout (default " << kTimeout << "s)\n"
              << "  --slave-address VALUE        Set the slave address (default " << kSlaveAddress << ")\n"
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
    serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout)));

    auto driver = std::make_unique<DefaultDriver>(std::move(serial));
    driver->set_slave_address(slave_address);

    std::cout << "Using the following parameters: " << std::endl;
    std::cout << " - port: " << port << std::endl;
    std::cout << " - baudrate: " << baudrate << "bps" << std::endl;
    std::cout << " - read/write timeout: " << timeout << "s" << std::endl;
    std::cout << " - slave address: " << slave_address << std::endl;

    std::cout << "Checking if the gripper is connected..." << std::endl;

    bool connected = driver->connect();
    if (!connected)
    {
      std::cout << "The gripper is not connected" << std::endl;
      return 1;
    }

    std::cout << "The gripper is connected." << std::endl;
    std::cout << "Grip the object..." << std::endl;

    driver->grip();

    std::cout << "Reading the gripper status..." << std::endl;

    epick_driver::GripperStatus status = driver->get_status();

    std::cout << "Status retrieved:" << std::endl;

    std::cout << " - gripper activation action: "
              << gripper_activation_action_to_string(status.gripper_activation_action) << std::endl;
    std::cout << " - gripper regulate action: " << gripper_regulate_action_to_string(status.gripper_regulate_action)
              << std::endl;
    std::cout << " - gripper mode: " << gripper_mode_to_string(status.gripper_mode) << std::endl;
    std::cout << " - object detection status: " << object_detection_to_string(status.object_detection_status)
              << std::endl;
    std::cout << " - gripper fault status: " << fault_status_to_string(status.gripper_fault_status) << std::endl;
    std::cout << " - actuator status: " << actuator_status_to_string(status.actuator_status) << std::endl;
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
