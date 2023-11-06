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

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

namespace
{
constexpr auto kBehaviorInterfacePackageName = "moveit_studio_behavior_interface";
constexpr auto kBehaviorBaseClassName = "moveit_studio::behaviors::SharedResourcesNodeLoaderBase";
constexpr auto kTestBehaviorName = "test_behavior_name";
}  // namespace

/**
 * @brief This test makes sure that the Behaviors provided in this package can be successfully registered and
 * instantiated by the behavior tree factory.
 */
TEST(EpickBehaviors, LoadBehaviorPlugins)
{
  pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      kBehaviorInterfacePackageName, kBehaviorBaseClassName);

  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("epick_moveit_studio::BehaviorLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }

  // Test that ClassLoader is able to find and instantiate each behavior using the package's plugin description info.
  EXPECT_NO_THROW((void)factory.instantiateTreeNode(kTestBehaviorName, "CompareEpickObjectDetectionStatus",
                                                    BT::NodeConfiguration()));
  EXPECT_NO_THROW(
      (void)factory.instantiateTreeNode(kTestBehaviorName, "GetEpickObjectDetectionStatus", BT::NodeConfiguration()));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
