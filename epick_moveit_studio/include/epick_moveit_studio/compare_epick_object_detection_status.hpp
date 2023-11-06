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

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/exceptions.h>
#include <epick_msgs/msg/object_detection_status.hpp>

namespace BT
{
/**
 * @brief Template specialization of convertFromString to parse a string as an epick_msgs::msg::ObjectDetectionStatus.
 * @details This allows setting one of the input ports of CompareEpickObjectDetectionStatus by typing a string.
 * @param str Input string to convert.
 * @return epick_msgs::msg::ObjectDetectionStatus
 */
template <>
inline epick_msgs::msg::ObjectDetectionStatus
convertFromString<epick_msgs::msg::ObjectDetectionStatus>(BT::StringView str)
{
  using Status = epick_msgs::msg::ObjectDetectionStatus;
  if (str == "OBJECT_DETECTED_AT_MIN_PRESSURE")
  {
    return epick_msgs::build<Status>().status(Status::OBJECT_DETECTED_AT_MIN_PRESSURE);
  }
  else if (str == "OBJECT_DETECTED_AT_MAX_PRESSURE")
  {
    return epick_msgs::build<Status>().status(Status::OBJECT_DETECTED_AT_MAX_PRESSURE);
  }
  else if (str == "NO_OBJECT_DETECTED")
  {
    return epick_msgs::build<Status>().status(Status::NO_OBJECT_DETECTED);
  }
  else if (str == "UNKNOWN")
  {
    return epick_msgs::build<Status>().status(Status::UNKNOWN);
  }
  else
  {
    throw BT::RuntimeError(std::string("To convert into a ObjectDetectionStatus message, the input string must be one "
                                       "of [OBJECT_DETECTED_AT_MIN_PRESSURE, OBJECT_DETECTED_AT_MAX_PRESSURE, "
                                       "NO_OBJECT_DETECTED, UNKNOWN]. Cannot parse the provided string: ")
                               .append(str));
  }
}
}  // namespace BT

namespace epick_moveit_studio
{
/**
 * @brief Behavior to compare ObjectDetectionStatus messages.
 * @details When ticked, gets two epick_msgs::msg::ObjectDetectionStatus messages from the "value1" and "value2" input
 * data ports. The behavior succeeds if they are identical and fails if they are different.
 *
 * | Data Port Name | Port Type | Object Type                            |
 * | -------------- | --------- | -------------------------------------- |
 * | value1         | input     | epick_msgs::msg::ObjectDetectionStatus |
 * | value2         | input     | epick_msgs::msg::ObjectDetectionStatus |
 */
class CompareEpickObjectDetectionStatus final : public BT::SyncActionNode
{
public:
  CompareEpickObjectDetectionStatus(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
}  // namespace epick_moveit_studio
