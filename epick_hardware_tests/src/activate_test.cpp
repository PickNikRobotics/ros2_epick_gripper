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

#include "epick_driver/default_command_interface.hpp"
#include "epick_driver/default_serial_interface.hpp"

#include <thread>
#include <chrono>
#include <memory>
#include <vector>
#include <iostream>

constexpr auto kSlaveAddress = 0x09;
constexpr auto kComPort = "/tmp/ttyVUSB0";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 2000;

int main(int argc, char* argv[])
{
  try
  {
    auto serial_interface = std::make_unique<epick_driver::DefaultSerialInterface>();
    serial_interface->set_port(kComPort);
    serial_interface->set_baudrate(kBaudRate);
    serial_interface->set_timeout(kTimeout);

    auto command_interface =
        std::make_unique<epick_driver::DefaultCommandInterface>(std::move(serial_interface), kSlaveAddress);

    bool connected = command_interface->connect();
    if (!connected)
    {
      std::cout << "Gripper not connected";
      return 1;
    }

    std::cout << "Deactivating gripper...\n";
    command_interface->deactivate();

    std::cout << "Activating gripper...\n";
    command_interface->activate();

    std::cout << "Gripper successfully activated.\n";
  }
  catch (const serial::IOException& e)
  {
    std::cout << "Failed to communicating with the Gripper:" << e.what();
    return 1;
  }

  return 0;
}
