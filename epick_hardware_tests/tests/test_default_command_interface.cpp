/*
 * Copyright (c) 2022, Giovanni Remigi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "epick_driver/default_command_interface.hpp"
#include "epick_driver/default_serial_interface.hpp"

#include <test_utils.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include <thread>
#include <chrono>
#include <memory>
#include <vector>

namespace epick_driver::test
{
constexpr auto kSlaveAddress = 0x09;
constexpr auto kComPort = "/tmp/ttyVUSB0";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 2000;

class TestDefaultCommandInterface : public ::testing::Test
{
public:
  void SetUp()
  {
    if (skip_test())
    {
      GTEST_SKIP() << "Skipping all tests for this fixture";
    }
    auto serial_interface = std::make_unique<epick_driver::DefaultSerialInterface>();
    serial_interface->set_port(kComPort);
    serial_interface->set_baudrate(kBaudRate);
    serial_interface->set_timeout(kTimeout);

    command_interface =
        std::make_unique<epick_driver::DefaultCommandInterface>(std::move(serial_interface), kSlaveAddress);
    command_interface->connect();
  }

  void TearDown()
  {
    if (command_interface)
    {
      command_interface->disconnect();
    }
  }

  std::unique_ptr<epick_driver::DefaultCommandInterface> command_interface = nullptr;
};

/**
 * Send an info command to the micro controller and verify the expected response.
 */
TEST_F(TestDefaultCommandInterface, activate)
{
  command_interface->activate();
}

}  // namespace epick_driver::test
