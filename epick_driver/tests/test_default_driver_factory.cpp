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

#include <gtest/gtest.h>

#include <epick_driver/default_driver_factory.hpp>

#include <mock/mock_driver.hpp>

#include <hardware_interface/hardware_info.hpp>

namespace epick_driver::test
{
// This factory will populate the injected mock with data read form the HardwareInfo.
class TestDriverFactory : public DefaultDriverFactory
{
public:
  explicit TestDriverFactory(std::unique_ptr<Driver> driver) : driver_{ std::move(driver) }
  {
  }

protected:
  std::unique_ptr<Driver> create_driver([[maybe_unused]] const hardware_interface::HardwareInfo& info) const override
  {
    return std::move(driver_);
  }

private:
  mutable std::unique_ptr<Driver> driver_;
};

/**
 * Here we test the driver factory with default parameters.
 */
TEST(TestDefaultDriverFactory, create_with_default_parameters)
{
  hardware_interface::HardwareInfo info;

  auto driver = std::make_unique<MockDriver>();

  // This line is only required when running the test inside the IDE.
  testing::Mock::AllowLeak(driver.get());

  EXPECT_CALL(*driver, set_slave_address(0x9));
  EXPECT_CALL(*driver, set_mode(GripperMode::AutomaticMode));
  EXPECT_CALL(*driver, set_grip_max_vacuum_pressure(-100));
  EXPECT_CALL(*driver, set_grip_min_vacuum_pressure(-10));
  EXPECT_CALL(*driver, set_grip_timeout(std::chrono::milliseconds(500)));
  EXPECT_CALL(*driver, set_release_timeout(std::chrono::milliseconds(500)));
  EXPECT_CALL(*driver, connect()).Times(0);
  EXPECT_CALL(*driver, disconnect()).Times(0);
  EXPECT_CALL(*driver, activate()).Times(0);
  EXPECT_CALL(*driver, deactivate()).Times(0);
  EXPECT_CALL(*driver, grip()).Times(0);
  EXPECT_CALL(*driver, release()).Times(0);
  EXPECT_CALL(*driver, get_status()).Times(0);

  TestDriverFactory driver_factory{ std::move(driver) };
  auto created_driver = driver_factory.create(info);
}

/**
 * Here we test the driver factory with given parameters.
 */
TEST(TestDefaultDriverFactory, create_with_given_parameters)
{
  hardware_interface::HardwareInfo info;

  info.hardware_parameters.emplace("slave_address", "1");
  info.hardware_parameters.emplace("mode", "AdvancedMode");
  info.hardware_parameters.emplace("grip_max_vacuum_pressure", "-50");
  info.hardware_parameters.emplace("grip_min_vacuum_pressure", "-10");
  info.hardware_parameters.emplace("grip_timeout", "1.0");
  info.hardware_parameters.emplace("release_timeout", "0.2");

  auto driver = std::make_unique<MockDriver>();

  // This line is only required when running the test inside the IDE.
  testing::Mock::AllowLeak(driver.get());

  EXPECT_CALL(*driver, set_slave_address(0x1));
  EXPECT_CALL(*driver, set_mode(GripperMode::AdvancedMode));
  EXPECT_CALL(*driver, set_grip_max_vacuum_pressure(-50));
  EXPECT_CALL(*driver, set_grip_min_vacuum_pressure(-10));
  EXPECT_CALL(*driver, set_grip_timeout(std::chrono::milliseconds(1000)));
  EXPECT_CALL(*driver, set_release_timeout(std::chrono::milliseconds(200)));
  EXPECT_CALL(*driver, connect()).Times(0);
  EXPECT_CALL(*driver, disconnect()).Times(0);
  EXPECT_CALL(*driver, activate()).Times(0);
  EXPECT_CALL(*driver, deactivate()).Times(0);
  EXPECT_CALL(*driver, grip()).Times(0);
  EXPECT_CALL(*driver, release()).Times(0);
  EXPECT_CALL(*driver, get_status()).Times(0);

  TestDriverFactory driver_factory{ std::move(driver) };
  auto created_driver = driver_factory.create(info);
}
}  // namespace epick_driver::test
