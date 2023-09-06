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

#pragma once

#include <gmock/gmock.h>

#include <epick_driver/driver.hpp>

#include <rclcpp/rclcpp.hpp>

namespace epick_driver::test
{
class MockDriver : public epick_driver::Driver
{
public:
  MOCK_METHOD(void, set_slave_address, (uint8_t slave_address), (override));
  MOCK_METHOD(void, set_mode, (GripperMode mode), (override));
  MOCK_METHOD(void, set_grip_max_vacuum_pressure, (float vacuum_pressure), (override));
  MOCK_METHOD(void, set_grip_min_vacuum_pressure, (float vacuum_pressure), (override));
  MOCK_METHOD(void, set_grip_timeout, (std::chrono::milliseconds grip_timeout), (override));
  MOCK_METHOD(void, set_release_timeout, (std::chrono::milliseconds release_timeout), (override));
  MOCK_METHOD(bool, connect, (), (override));
  MOCK_METHOD(void, disconnect, (), (override));
  MOCK_METHOD(void, activate, (), (override));
  MOCK_METHOD(void, deactivate, (), (override));
  MOCK_METHOD(void, grip, (), (override));
  MOCK_METHOD(void, release, (), (override));
  MOCK_METHOD(GripperStatus, get_status, (), (override));
};
}  // namespace epick_driver::test
