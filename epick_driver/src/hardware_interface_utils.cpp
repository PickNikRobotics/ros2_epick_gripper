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

#include <algorithm>

#include <epick_driver/hardware_interface_utils.hpp>

namespace epick_driver::hardware_interface_utils
{
/**
 * Look for a GPIO interface in the hardware info.
 * @param gpio_name The component name.
 * @param interface_name The interface name.
 * @return the interface info of the requested interface, if any.
 */
std::optional<hardware_interface::InterfaceInfo>
get_gpios_command_interface(std::string gpio_name, std::string interface_name,
                            const hardware_interface::HardwareInfo& info)
{
  auto gpio_component = std::find_if(info.gpios.begin(), info.gpios.end(),
                                     [&gpio_name](const auto& gpio) { return gpio.name == gpio_name; });

  if (gpio_component != info.gpios.end())
  {
    auto hardware_info =
        std::find_if(gpio_component->command_interfaces.begin(), gpio_component->command_interfaces.end(),
                     [&interface_name](const auto& cmd) { return cmd.name == interface_name; });

    if (hardware_info != gpio_component->command_interfaces.end())
    {
      return *hardware_info;
    }
  }
  return std::nullopt;
}

std::optional<hardware_interface::InterfaceInfo> get_gpios_state_interface(std::string gpio_name,
                                                                           std::string interface_name,
                                                                           const hardware_interface::HardwareInfo& info)
{
  auto gpio_component = std::find_if(info.gpios.begin(), info.gpios.end(),
                                     [&gpio_name](const auto& gpio) { return gpio.name == gpio_name; });

  if (gpio_component != info.gpios.end())
  {
    auto hardware_info = std::find_if(gpio_component->state_interfaces.begin(), gpio_component->state_interfaces.end(),
                                      [&interface_name](const auto& cmd) { return cmd.name == interface_name; });

    if (hardware_info != gpio_component->state_interfaces.end())
    {
      return *hardware_info;
    }
  }
  return std::nullopt;
}

bool is_true(double value)
{
  return value >= 0.5;
}

bool is_false(double value)
{
  return value < 0.5;
}

std::optional<hardware_interface::InterfaceInfo>
get_joints_command_interface(std::string joint_name, std::string interface_name,
                             const hardware_interface::HardwareInfo& info)
{
  auto joint_component = std::find_if(info.joints.begin(), info.joints.end(),
                                      [&joint_name](const auto& gpio) { return gpio.name == joint_name; });

  if (joint_component != info.joints.end())
  {
    auto hardware_info =
        std::find_if(joint_component->command_interfaces.begin(), joint_component->command_interfaces.end(),
                     [&interface_name](const auto& cmd) { return cmd.name == interface_name; });

    if (hardware_info != joint_component->command_interfaces.end())
    {
      return *hardware_info;
    }
  }
  return std::nullopt;
}

std::optional<hardware_interface::InterfaceInfo>
get_joints_state_interface(std::string joint_name, std::string interface_name,
                           const hardware_interface::HardwareInfo& info)
{
  auto joint_component = std::find_if(info.joints.begin(), info.joints.end(),
                                      [&joint_name](const auto& gpio) { return gpio.name == joint_name; });

  if (joint_component != info.joints.end())
  {
    auto hardware_info =
        std::find_if(joint_component->state_interfaces.begin(), joint_component->state_interfaces.end(),
                     [&interface_name](const auto& cmd) { return cmd.name == interface_name; });

    if (hardware_info != joint_component->state_interfaces.end())
    {
      return *hardware_info;
    }
  }
  return std::nullopt;
}

}  // namespace epick_driver::hardware_interface_utils
