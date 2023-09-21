/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Fraunhofer IPA nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <moveit_ros_control_interface/ControllerHandle.h>
#include <pluginlib/class_list_macros.hpp>
#include <moveit_simple_controller_manager/gripper_controller_handle.h>
#include <rclcpp/node.hpp>
#include <memory>

namespace
{
constexpr auto kGripperCommandAction = "gripper_cmd";
}  // namespace

namespace epick_moveit_plugin
{
/**
 * \brief Controller allocator plugin to allow the EpickGripperActionController to execute trajectories using MoveIt in
 * the same way as the ros2_controllers GripperActionController
 */
class EpickGripperControllerAllocator : public moveit_ros_control_interface::ControllerHandleAllocator
{
public:
  moveit_controller_manager::MoveItControllerHandlePtr alloc(const rclcpp::Node::SharedPtr& node,
                                                             const std::string& name,
                                                             const std::vector<std::string>& /* resources */) override
  {
    return std::make_shared<moveit_simple_controller_manager::GripperControllerHandle>(node, name,
                                                                                       kGripperCommandAction);
  }
};

}  // namespace epick_moveit_plugin

PLUGINLIB_EXPORT_CLASS(epick_moveit_plugin::EpickGripperControllerAllocator,
                       moveit_ros_control_interface::ControllerHandleAllocator);
