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

#include <optional>
#include <string>

#include <hardware_interface/hardware_info.hpp>

namespace epick_driver::hardware_interface_utils
{
/**
 * Look for a GPIO command interface in the hardware info.
 * @param gpio_name The component name.
 * @param interface_name The interface name.
 * @return the interface info of the requested interface, if any.
 */
std::optional<hardware_interface::InterfaceInfo>
get_gpios_command_interface(std::string gpio_name, std::string interface_name,
                            const hardware_interface::HardwareInfo& info);

/**
 * Look for a GPIO state interface in the hardware info.
 * @param gpio_name The component name.
 * @param interface_name The interface name.
 * @return the interface info of the requested interface, if any.
 */
std::optional<hardware_interface::InterfaceInfo>
get_gpios_state_interface(std::string gpio_name, std::string interface_name,
                          const hardware_interface::HardwareInfo& info);

/**
 * Look for a joint command interface in the hardware info.
 * @param joint_name The component name.
 * @param interface_name The interface name.
 * @return the interface info of the requested interface, if any.
 */
std::optional<hardware_interface::InterfaceInfo>
get_joints_command_interface(std::string joint_name, std::string interface_name,
                             const hardware_interface::HardwareInfo& info);

/**
 * Look for a joint state interface in the hardware info.
 * @param joint_name The component name.
 * @param interface_name The interface name.
 * @return the interface info of the requested interface, if any.
 */
std::optional<hardware_interface::InterfaceInfo>
get_joints_state_interface(std::string joint_name, std::string interface_name,
                           const hardware_interface::HardwareInfo& info);

/**
 * All command and state interfaces work with double values. We can
 * use double value to represent boolean values:
 * 0.0 = true
 * 1.0 = false
 * To avoid directly comparing double, to determine if a double represents
 * a false or a true, we use a simple formula:
 * bool is_true = value >= 0.5
 * @param value The value to be converted into a boolean.
 * @return The boolean linked to the given double value.
 */
bool is_true(double value);

/**
 * All command and state interfaces work with double values. We can
 * use double value to represent boolean values:
 * 0.0 = true
 * 1.0 = false
 * To avoid directly comparing double, to determine if a double represents
 * a false or a true, we use a simple formula:
 * bool is_true = value >= 0.5
 * @param value The value to be converted into a boolean.
 * @return The boolean linked to the given double value.
 */
bool is_false(double value);
}  // namespace epick_driver::hardware_interface_utils
