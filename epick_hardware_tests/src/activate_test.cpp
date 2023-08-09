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

#include "serial/serial.h"
#include "epick_driver/crc_utils.hpp"
#include "epick_driver/data_utils.hpp"

#include <memory>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 115200;

int main()
{
  try
  {
    auto serial_interface = std::make_unique<serial::Serial>();
    serial_interface->setPort(kComPort);
    serial_interface->setBaudrate(kBaudRate);

    // Activate.
    std::vector<uint8_t> activate_request{
      0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // Set pressure level.
    std::vector<uint8_t> set_pressure_level_request{ 0x09, 0x10, 0x03, 0xE9, 0x00, 0x02, 0x04, 0x00, 0x19, 0x0A, 0x23 };

    auto request = activate_request;

    // The CRC-16 is correct, I've checked many times with available examples and
    // online tools.
    auto crc16 = epick_driver::crc_utils::compute_crc(request);
    request.push_back(epick_driver::data_utils::get_msb(crc16));
    request.push_back(epick_driver::data_utils::get_lsb(crc16));

    std::cout << "Request: " << epick_driver::data_utils::to_hex(request) << std::endl;

    serial_interface->open();
    bool open = serial_interface->isOpen();
    if (!open)
    {
      std::cout << "Gripper not connected" << std::endl;
      return 1;
    }

    std::cout << "Gripper connected." << std::endl;
    std::cout << "Sending request..." << std::endl;

    serial_interface->write(request);
    serial_interface->flush();

    std::cout << "Reading response..." << std::endl;

    std::vector<uint8_t> response;
    size_t response_size = 8;
    size_t num_bytes_read = serial_interface->read(response, response_size);

    std::cout << "Response received: " << epick_driver::data_utils::to_hex(response) << std::endl;

    if (num_bytes_read != response_size)
    {
      std::cout
          << "Requested " + std::to_string(response_size) + " bytes, but only got " + std::to_string(num_bytes_read)
          << std::endl;
      return 1;
    }

    std::cout << "Gripper successfully activated." << std::endl;
  }
  catch (const serial::IOException& e)
  {
    std::cout << "Failed to communicating with the Gripper:" << e.what();
    return 1;
  }

  return 0;
}
