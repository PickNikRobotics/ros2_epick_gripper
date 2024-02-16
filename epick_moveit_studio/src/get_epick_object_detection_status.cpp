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

#include <epick_moveit_studio/get_epick_object_detection_status.hpp>

#include <epick_msgs/msg/object_detection_status.hpp>
#include <moveit_studio_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace
{
/** @brief Maximum duration to wait for a message to be published before failing. */
constexpr auto kWaitDuration = std::chrono::seconds{ 1 };
}  // namespace

namespace epick_moveit_studio
{
GetEpickObjectDetectionStatus::GetEpickObjectDetectionStatus(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<epick_msgs::msg::ObjectDetectionStatus>(name, config,
                                                                                                      shared_resources)
{
}

tl::expected<std::chrono::duration<double>, std::string> GetEpickObjectDetectionStatus::getWaitForMessageTimeout()
{
  return kWaitDuration;
}

}  // namespace epick_moveit_studio

template class moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<epick_msgs::msg::ObjectDetectionStatus>;
