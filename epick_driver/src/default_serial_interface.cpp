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

#include "epick_driver/default_serial_interface.hpp"

#include <serial/serial.h>

#include <epick_driver/default_serial_interface.hpp>

namespace epick_driver
{
DefaultSerialInterface::DefaultSerialInterface() : serial_{ std::make_unique<serial::Serial>() }
{
}

void DefaultSerialInterface::open()
{
  serial_->open();
}

bool DefaultSerialInterface::is_open() const
{
  return serial_->isOpen();
}

void DefaultSerialInterface::close()
{
  serial_->close();
}

std::vector<uint8_t> DefaultSerialInterface::read(size_t size)
{
  std::vector<uint8_t> data;
  size_t num_bytes_read = serial_->read(data, size);
  if (num_bytes_read != size)
  {
    const auto error_msg =
        "Requested " + std::to_string(size) + " bytes, but only got " + std::to_string(num_bytes_read);
    THROW(serial::IOException, error_msg.c_str());
  }

  return data;
}

void DefaultSerialInterface::write(const std::vector<uint8_t>& data)
{
  std::size_t num_bytes_written = serial_->write(data);
  serial_->flush();
  if (num_bytes_written != data.size())
  {
    const auto error_msg = "Attempted to write " + std::to_string(data.size()) + " bytes, but only wrote " +
                           std::to_string(num_bytes_written);
    THROW(serial::IOException, error_msg.c_str());
  }
}

void DefaultSerialInterface::set_port(const std::string& port)
{
  serial_->setPort(port);
}

std::string DefaultSerialInterface::get_port() const
{
  return serial_->getPort();
}

void DefaultSerialInterface::set_timeout(uint32_t timeout_ms)
{
  timeout_ms_ = timeout_ms;
  serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
  serial_->setTimeout(timeout);
}

uint32_t DefaultSerialInterface::get_timeout() const
{
  return timeout_ms_;
}

void DefaultSerialInterface::set_baudrate(uint32_t baudrate)
{
  serial_->setBaudrate(baudrate);
}

uint32_t DefaultSerialInterface::get_baudrate() const
{
  return serial_->getBaudrate();
}

}  // namespace epick_driver
