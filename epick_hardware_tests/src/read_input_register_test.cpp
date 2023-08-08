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

#include <memory>
#include <vector>
#include <iostream>

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 9600;

int main()
{
  try
  {
    auto serial_interface = std::make_unique<serial::Serial>();
    serial_interface->setPort(kComPort);
    serial_interface->setBaudrate(kBaudRate);

    const std::vector<uint8_t> request{ 0x09, 0x04, 0x07, 0xD0, 0x00, 0x02, 0x70, 0x0E };
    const std::vector<uint8_t> expected_response{ 0x09, 0x04, 0x04, 0xF0, 0x00, 0x00, 0x00, 0x41, 0x44 };

    serial_interface->open();
    bool open = serial_interface->isOpen();
    if (!open)
    {
      std::cout << "Gripper not connected" << std::endl;
      return 1;
    }

    std::cout << "Gripper connected. Sending command..." << std::endl;

    serial_interface->write(request);
    serial_interface->flush();

    std::cout << "Reading response..." << std::endl;

    std::vector<uint8_t> response;
    size_t num_bytes_read = serial_interface->read(response, expected_response.size());

    if (num_bytes_read != expected_response.size())
    {
      std::cout << "Requested " + std::to_string(expected_response.size()) + " bytes, but only got " +
                       std::to_string(num_bytes_read)
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
