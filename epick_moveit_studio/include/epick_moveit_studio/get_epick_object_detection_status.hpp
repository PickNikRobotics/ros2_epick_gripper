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

#include <behaviortree_cpp/exceptions.h>
#include <epick_msgs/msg/detail/object_detection_status__builder.hpp>
#include <epick_msgs/msg/detail/object_detection_status__struct.hpp>
#include <moveit_studio_behavior_interface/get_message_from_topic.hpp>

#include <epick_msgs/msg/object_detection_status.hpp>

namespace BT
{
/**
 * @brief Template specialization of convertToString to split a string into an epick_msgs::msg::ObjectDetectionStatus.
 * @param str Input string to convert.
 * @return epick_msgs::msg::ObjectDetectionStatus
 */
template <>
inline epick_msgs::msg::ObjectDetectionStatus convertFromString<epick_msgs::msg::ObjectDetectionStatus>(BT::StringView str)
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
    throw BT::RuntimeError(std::string("Invalid input: ").append(str));
  }
}
}  // namespace BT

namespace epick_moveit_studio
{
/**
 * @brief Capture a point cloud. The name of the topic containing the point cloud is set through the
 * "topic_name" parameter, and the resulting point cloud is available on the "message_out" output
 * port.
 *
 * @details
 * | Data Port Name | Port Type | Object Type                   |
 * | -------------- | --------- | ----------------------------- |
 * | topic_name     | input     | std::string                   |
 * | message_out    | output    | sensor_msgs::msg::PointCloud2 |
 */
class GetEpickObjectDetectionStatus final : public moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<epick_msgs::msg::ObjectDetectionStatus>
{
public:
  GetEpickObjectDetectionStatus(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

private:
  /** @brief Override getWaitForMessageTimeout to allow failing if no point cloud is received within the expected duration. */
  // fp::Result<std::chrono::duration<double>> getWaitForMessageTimeout() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<fp::Result<bool>> future_;
};
}  // namespace epick_moveit_studio
