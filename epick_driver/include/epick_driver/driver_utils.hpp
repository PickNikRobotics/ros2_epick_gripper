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

#include "epick_driver/driver.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>

namespace epick_driver::driver_utils
{
// clang-format off

// Associate a configuration of bits to GripperActivation enum.
const std::unordered_map<uint8_t, GripperActivation>& gACT_lookup()
{
  static const std::unordered_map<uint8_t, GripperActivation> map{
    { 0b0, GripperActivation::Inactive },
    { 0b1, GripperActivation::Active },
    };
  return map;
}

// Associate a configuration of bits to Regulate enum.
const std::unordered_map<uint8_t, Regulate>& gGTO_lookup()
{
  static const std::unordered_map<uint8_t, Regulate> map{};
  return map;
}

// Associate a configuration of bits to GripperMode enum.
const std::unordered_map<uint8_t, GripperMode>& gMOD_lookup()
{
  static const std::unordered_map<uint8_t, GripperMode> map{
    { 0b00, GripperMode::AutomaticMode },
    { 0b01, GripperMode::AdvancedMode },
    { 0b10, GripperMode::Reserved },
    { 0b11, GripperMode::Reserved } };
  return map;
}

// Associate a configuration of bits to GripperMode enum.
const std::unordered_map<uint8_t, ObjectDetection> gOBJ_lookup()
{
  static const std::unordered_map<uint8_t, ObjectDetection> map{
    { 0b00, ObjectDetection::Unknown },
    { 0b01, ObjectDetection::ObjectDetected },
    { 0b10, ObjectDetection::ObjectDetected },
    { 0b11, ObjectDetection::NoObjectDetected } };
  return map;
}

// Associate a configuration of bits to FaultStatus enum.
const std::unordered_map<uint8_t, FaultStatus>  gFLT_lookup()
{
  static const std::unordered_map<uint8_t, FaultStatus> map{
    { 0x0, FaultStatus::NoFault },
    { 0x1, FaultStatus::Unknown },
    { 0x2, FaultStatus::Unknown },
    { 0x3, FaultStatus::PorousMaterialDetected },
    { 0x4, FaultStatus::Unknown },
    { 0x5, FaultStatus::AcionDelayed },
    { 0x6, FaultStatus::GrippingTimeout },
    { 0x7, FaultStatus::ActivationBitNotSet },
    { 0x8, FaultStatus::MaximumTemperatureExceeded },
    { 0x9, FaultStatus::NoCommunicationForAtLeastOneSecond },
    { 0xA, FaultStatus::UderMinimumOperatingVoltage },
    { 0xB, FaultStatus::AutomaticReleaseInProgress },
    { 0XC, FaultStatus::InternalFault },
    { 0xD, FaultStatus::Unknown },
    { 0xE, FaultStatus::Unknown },
    { 0xF, FaultStatus::AutomaticReleaseCompleted } };
  return map;
}

// Associate a configuration of bits to ActuatorStatus enum.
const std::unordered_map<uint8_t, ActuatorStatus> gVAS_lookup()
{
  static const std::unordered_map<uint8_t, ActuatorStatus> map{
    { 0b00, ActuatorStatus::Standby },
    { 0b01, ActuatorStatus::Gripping },
    { 0b10, ActuatorStatus::PassiveReleasing },
    { 0b11, ActuatorStatus::ActiveReleasing } };
  return map;
}

// clang-format on

/**
 * Convert a GripperActivation enum into a string.
 * @param gripper_activation The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_activation_to_string(const GripperActivation gripper_activation)
{
  static std::map<GripperActivation, std::string> map = {
    { GripperActivation::Inactive, "Inactive" },
    { GripperActivation::Active, "Active" },
  };
  return map.at(gripper_activation);
}

/**
 * Convert an ObjectDetection enum into a string.
 * @param object_detection The enum.
 * @return A string representation of the given enum
 */
const std::string object_detection_to_string(const ObjectDetection object_detection)
{
  static std::map<ObjectDetection, std::string> map = {
    { ObjectDetection::Unknown, "Unknown" },
    { ObjectDetection::ObjectDetected, "ObjectDetected" },
    { ObjectDetection::NoObjectDetected, "NoObjectDetected" },
  };
  return map.at(object_detection);
}

/**
 * Convert a GripperMode enum into a string.
 * @param gripper_mode The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_mode_to_string(const GripperMode gripper_mode)
{
  static std::map<GripperMode, std::string> map = { { GripperMode::AutomaticMode, "AutomaticMode" },
                                                    { GripperMode::AdvancedMode, "AdvancedMode" },
                                                    { GripperMode::Reserved, "Reserved" } };
  return map.at(gripper_mode);
}

/**
 * Convert a FaultStatus enum into a string.
 * @param fault_status The enum.
 * @return A string representation of the given enum
 */
const std::string fault_status_to_string(const FaultStatus fault_status)
{
  static std::map<epick_driver::FaultStatus, std::string> map = {
    { epick_driver::FaultStatus::NoFault, "NoFault" },
    { epick_driver::FaultStatus::AcionDelayed, "AcionDelayed" },
    { epick_driver::FaultStatus::PorousMaterialDetected, "PorousMaterialDetected" },
    { epick_driver::FaultStatus::GrippingTimeout, "GrippingTimeout" },
    { epick_driver::FaultStatus::ActivationBitNotSet, "ActivationBitNotSet" },
    { epick_driver::FaultStatus::MaximumTemperatureExceeded, "MaximumTemperatureExceeded" },
    { epick_driver::FaultStatus::NoCommunicationForAtLeastOneSecond, "NoCommunicationForAtLeastOneSecond" },
    { epick_driver::FaultStatus::UderMinimumOperatingVoltage, "UderMinimumOperatingVoltage" },
    { epick_driver::FaultStatus::AutomaticReleaseInProgress, "AutomaticReleaseInProgress" },
    { epick_driver::FaultStatus::InternalFault, "InternalFault" },
    { epick_driver::FaultStatus::AutomaticReleaseCompleted, "AutomaticReleaseCompleted" },
    { epick_driver::FaultStatus::Unknown, "Unknown" },
  };
  return map.at(fault_status);
}

/**
 * Convert a ActuatorStatus enum into a string.
 * @param actuator_status The enum.
 * @return A string representation of the given enum
 */
const std::string actuator_status_to_string(const ActuatorStatus actuator_status)
{
  static std::map<ActuatorStatus, std::string> map = { { ActuatorStatus::Standby, "Standby" },
                                                       { ActuatorStatus::Gripping, "Gripping" },
                                                       { ActuatorStatus::PassiveReleasing, "PassiveReleasing" },
                                                       { ActuatorStatus::ActiveReleasing, "ActiveReleasing" } };
  return map.at(actuator_status);
}

};  // namespace epick_driver::driver_utils
