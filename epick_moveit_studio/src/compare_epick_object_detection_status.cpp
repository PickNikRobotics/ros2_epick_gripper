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

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <epick_moveit_studio/compare_epick_object_detection_status.hpp>
#include <epick_msgs/msg/object_detection_status.hpp>

namespace
{
constexpr auto kPortIDValue1 = "value1";
constexpr auto kPortIDValue2 = "value2";
}  // namespace

namespace epick_moveit_studio
{
CompareEpickObjectDetectionStatus::CompareEpickObjectDetectionStatus(const std::string& name,
                                                                     const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList CompareEpickObjectDetectionStatus::providedPorts()
{
  return BT::PortsList{
    BT::InputPort<epick_msgs::msg::ObjectDetectionStatus>(kPortIDValue1),
    BT::InputPort<epick_msgs::msg::ObjectDetectionStatus>(kPortIDValue2),
  };
}

BT::NodeStatus CompareEpickObjectDetectionStatus::tick()
{
  const auto value1 = getInput<epick_msgs::msg::ObjectDetectionStatus>(kPortIDValue1);
  const auto value2 = getInput<epick_msgs::msg::ObjectDetectionStatus>(kPortIDValue2);

  if (!value1.has_value() || !value2.has_value())
  {
    return BT::NodeStatus::FAILURE;
  }

  if (value1.value().status == value2.value().status)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace epick_moveit_studio
