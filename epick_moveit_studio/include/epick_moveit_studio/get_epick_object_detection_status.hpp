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

#include <epick_msgs/msg/object_detection_status.hpp>
#include <moveit_studio_behavior_interface/get_message_from_topic.hpp>

namespace epick_moveit_studio
{
/**
 * @brief Capture a epick_msgs::msg::ObjectDetectionStatus message.
 * @details The topic to monitor is set through the "topic_name" parameter, and the resulting message is available on
 * the "message_out" output port.
 *
 * @details
 * | Data Port Name | Port Type | Object Type                            |
 * | -------------- | --------- | -------------------------------------- |
 * | topic_name     | input     | std::string                            |
 * | message_out    | output    | epick_msgs::msg::ObjectDetectionStatus |
 */
class GetEpickObjectDetectionStatus final
  : public moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<epick_msgs::msg::ObjectDetectionStatus>
{
public:
  GetEpickObjectDetectionStatus(const std::string& name, const BT::NodeConfiguration& config,
                                const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

private:
  tl::expected<std::chrono::duration<double>, std::string> getWaitForMessageTimeout() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace epick_moveit_studio
