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

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/exceptions.h>
#include <epick_moveit_studio/compare_epick_object_detection_status.hpp>
#include <epick_msgs/msg/object_detection_status.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
constexpr auto kPortIDValue1 = "value1";
constexpr auto kPortIDValue2 = "value2";

using ObjectDetectionStatus = epick_msgs::msg::ObjectDetectionStatus;
using ::testing::Eq;
using ::testing::Throw;
}  // namespace

namespace epick_moveit_studio
{
class CompareEpickObjectDetectionStatusTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    BT::NodeConfiguration config;
    for (const auto& port : CompareEpickObjectDetectionStatus::providedPorts())
    {
      if (port.second.direction() == BT::PortDirection::INPUT || port.second.direction() == BT::PortDirection::INOUT)
      {
        config.input_ports.try_emplace(port.first, "=");
      }
      if (port.second.direction() == BT::PortDirection::OUTPUT || port.second.direction() == BT::PortDirection::INOUT)
      {
        config.output_ports.try_emplace(port.first, "=");
      }
    }

    config.blackboard = BT::Blackboard::create();
    blackboard_ptr = config.blackboard.get();
    behavior = std::make_unique<CompareEpickObjectDetectionStatus>("CompareEpickObjectDetectionStatus", config);
  }

  BT::Blackboard* blackboard_ptr;
  std::unique_ptr<CompareEpickObjectDetectionStatus> behavior;
};

TEST_F(CompareEpickObjectDetectionStatusTest, FailureIfValue1Missing)
{
  // GIVEN the input port for the second value is not set

  // GIVEN valid input for the first value
  blackboard_ptr->set(kPortIDValue1, ObjectDetectionStatus());

  // WHEN the behavior is ticked
  // THEN the behavior fails
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(CompareEpickObjectDetectionStatusTest, FailureIfValue2Missing)
{
  // GIVEN the input port for the first value is not set

  // GIVEN valid input for the second value
  blackboard_ptr->set(kPortIDValue2, ObjectDetectionStatus());

  // WHEN the behavior is ticked
  // THEN the behavior fails
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(CompareEpickObjectDetectionStatusTest, SucceedsIfValuesMatch)
{
  // GIVEN valid and matching inputs
  blackboard_ptr->set(kPortIDValue1,
                      epick_msgs::build<ObjectDetectionStatus>().status(ObjectDetectionStatus::NO_OBJECT_DETECTED));
  blackboard_ptr->set(kPortIDValue2,
                      epick_msgs::build<ObjectDetectionStatus>().status(ObjectDetectionStatus::NO_OBJECT_DETECTED));

  // WHEN the behavior is ticked
  // THEN it succeeds
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(CompareEpickObjectDetectionStatusTest, SucceedsIfValuesMatchStringConversion)
{
  // GIVEN valid inputs which are equivalent to the same value, where one is an ObjectDetectionStatus message and the other is a string
  blackboard_ptr->set(kPortIDValue1, epick_msgs::build<ObjectDetectionStatus>().status(
                                         ObjectDetectionStatus::OBJECT_DETECTED_AT_MAX_PRESSURE));
  blackboard_ptr->set(kPortIDValue2, "OBJECT_DETECTED_AT_MAX_PRESSURE");

  // WHEN the behavior is ticked
  // THEN it succeeds
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(CompareEpickObjectDetectionStatusTest, FailureIfValuesMismatch)
{
  // GIVEN valid and different inputs
  blackboard_ptr->set(kPortIDValue1,
                      epick_msgs::build<ObjectDetectionStatus>().status(ObjectDetectionStatus::NO_OBJECT_DETECTED));
  blackboard_ptr->set(kPortIDValue2, epick_msgs::build<ObjectDetectionStatus>().status(
                                         ObjectDetectionStatus::OBJECT_DETECTED_AT_MIN_PRESSURE));

  // WHEN the behavior is ticked
  // THEN it fails
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::FAILURE);
}

TEST(CompareEpickObjectDetectionStatus, TestConvertFromString)
{
  // WHEN we provide strings corresponding to the status enums
  // THEN each string is converted into the correct enum
  EXPECT_THAT(BT::convertFromString<ObjectDetectionStatus>("NO_OBJECT_DETECTED"),
              Eq(epick_msgs::build<ObjectDetectionStatus>().status(ObjectDetectionStatus::NO_OBJECT_DETECTED)));
  EXPECT_THAT(
      BT::convertFromString<ObjectDetectionStatus>("OBJECT_DETECTED_AT_MIN_PRESSURE"),
      Eq(epick_msgs::build<ObjectDetectionStatus>().status(ObjectDetectionStatus::OBJECT_DETECTED_AT_MIN_PRESSURE)));
  EXPECT_THAT(
      BT::convertFromString<ObjectDetectionStatus>("OBJECT_DETECTED_AT_MAX_PRESSURE"),
      Eq(epick_msgs::build<ObjectDetectionStatus>().status(ObjectDetectionStatus::OBJECT_DETECTED_AT_MAX_PRESSURE)));
  EXPECT_THAT(BT::convertFromString<ObjectDetectionStatus>("UNKNOWN"),
              Eq(epick_msgs::build<ObjectDetectionStatus>().status(ObjectDetectionStatus::UNKNOWN)));

  // WHEN we provide a string that does not correspond to any status enum
  // THEN an exception is thrown
  EXPECT_THROW((void)BT::convertFromString<ObjectDetectionStatus>("some_nonsensical_input"), BT::RuntimeError);
}
}  // namespace epick_moveit_studio
