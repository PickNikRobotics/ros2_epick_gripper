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
#include <cmath>
#include <iostream>
#include <map>
#include <unordered_map>

#include <epick_driver/default_driver_utils.hpp>

namespace epick_driver::default_driver_utils
{

/**
 * Sets bits in a register based on a bitmask and a set of bits.
 * @param reg Initial register value.
 * @param bitmask Mask that indicates which bits in the register should be modified.
 *        A '1' in a bit position indicates that the corresponding bit in the register
 *        will be modified, and a '0' means it will remain unchanged.
 * @param bits Bits to be set in the register. Only the bits that are '1' in the bitmask
 *        will be set in the register. Other bits will be ignored.
 */
void set_bits(uint8_t& reg, uint8_t bitmask, uint8_t bits)
{
  reg &= ~bitmask;
  reg |= (bits & bitmask);
}

///////////////////////////////////////////////////////////////////////////////
/// Gripper activation request.
///

constexpr uint8_t gACT_mask = 0b00000001;

void set_gripper_activation_action(uint8_t& reg, const GripperActivationAction gripper_activation_action)
{
  switch (gripper_activation_action)
  {
    case GripperActivationAction::ClearGripperFaultStatus:
      set_bits(reg, gACT_mask, 0b00000000);
      break;
    case GripperActivationAction::Activate:
      set_bits(reg, gACT_mask, 0b00000001);
      break;
    default:
      break;
  }
}

GripperActivationAction get_gripper_activation_action(const uint8_t& reg)
{
  // clang-format off
  static const std::unordered_map<uint8_t, GripperActivationAction> map{
    { 0b00000000, GripperActivationAction::ClearGripperFaultStatus },
    { 0b00000001, GripperActivationAction::Activate } };
  // clang-format on
  return map.at(reg & default_driver_utils::gACT_mask);
}

const std::string gripper_activation_action_to_string(const GripperActivationAction gripper_activation_action)
{
  // clang-format off
  static std::map<GripperActivationAction, std::string> map = {
    { GripperActivationAction::ClearGripperFaultStatus, "ClearGripperFaultStatus" },
    { GripperActivationAction::Activate, "Activate" },
    };
  // clang-format on
  return map.at(gripper_activation_action);
}

///////////////////////////////////////////////////////////////////////////////
/// Gripper mode.
///

constexpr uint8_t gMOD_mask = 0b00000110;

void set_gripper_mode(uint8_t& reg, const GripperMode gripper_mode)
{
  switch (gripper_mode)
  {
    case GripperMode::AutomaticMode:
      set_bits(reg, gMOD_mask, 0b00000000);
      break;
    case GripperMode::AdvancedMode:
      set_bits(reg, gMOD_mask, 0b00000010);
      break;
    default:
      break;
  }
}

GripperMode get_gripper_mode(const uint8_t& reg)
{
  // clang-format off
  static const std::unordered_map<uint8_t, GripperMode> map{
    { 0b00000000, GripperMode::AutomaticMode },
    { 0b00000010, GripperMode::AdvancedMode },
    { 0b00000100, GripperMode::Unknown },
    { 0b00000110, GripperMode::Unknown } };
  // clang-format on
  return map.at(reg & default_driver_utils::gMOD_mask);
}

const std::string gripper_mode_to_string(const GripperMode gripper_mode)
{
  // clang-format off
  static std::map<GripperMode, std::string> map = {
    { GripperMode::AutomaticMode, "AutomaticMode" },
    { GripperMode::AdvancedMode, "AdvancedMode" },
    { GripperMode::Unknown, "Unknown" } };
  // clang-format on
  return map.at(gripper_mode);
}

///////////////////////////////////////////////////////////////////////////////
/// Regulate.
///

constexpr uint8_t gGTO_mask = 0b00001000;

void set_gripper_regulate_action(uint8_t& reg, const GripperRegulateAction regulate_action)
{
  switch (regulate_action)
  {
    case GripperRegulateAction::StopVacuumGenerator:
      set_bits(reg, gGTO_mask, 0b00000000);
      break;
    case GripperRegulateAction::FollowRequestedVacuumParameters:
      set_bits(reg, gGTO_mask, 0b00001000);
      break;
    default:
      break;
  }
}

GripperRegulateAction get_gripper_regulate_action(const uint8_t& reg)
{
  // clang-format off
  static const std::unordered_map<uint8_t, GripperRegulateAction> map{
    { 0b00000000, GripperRegulateAction::StopVacuumGenerator },
    { 0b00001000, GripperRegulateAction::FollowRequestedVacuumParameters } };
  // clang-format on
  return map.at(reg & default_driver_utils::gGTO_mask);
}

const std::string gripper_regulate_action_to_string(const GripperRegulateAction gripper_regulate_action)
{
  // clang-format off
  static std::map<GripperRegulateAction, std::string> map = {
    { GripperRegulateAction::StopVacuumGenerator, "StopVacuumGenerator" },
    { GripperRegulateAction::FollowRequestedVacuumParameters, "FollowRequestedVacuumParameters" } };
  // clang-format on
  return map.at(gripper_regulate_action);
}

GripperRegulateAction double_to_regulate_action(double regulate)
{
  if (regulate >= 0.5)
  {
    return GripperRegulateAction::FollowRequestedVacuumParameters;
  }
  else
  {
    return GripperRegulateAction::StopVacuumGenerator;
  }
}

double regulate_action_to_double(GripperRegulateAction regulate)
{
  if (regulate == GripperRegulateAction::FollowRequestedVacuumParameters)
  {
    return 1.0;
  }
  else
  {
    return 0.0;
  }
}

///////////////////////////////////////////////////////////////////////////////
/// Automatic release action.
///

constexpr uint8_t gATR_mask = 0b00010000;

/** Associate a configuration of bits to the GripperAutomaticReleaseAction enum. */
const std::unordered_map<uint8_t, GripperReleaseAction>& gATR_lookup()
{
  // clang-format off
  static const std::unordered_map<uint8_t, GripperReleaseAction> map{
    { 0b00000000, GripperReleaseAction::NormalRelease },
    { 0b00010000, GripperReleaseAction::ReleaseWithoutTimeout } };
  // clang-format on
  return map;
}

void set_gripper_automatic_release_action(uint8_t& reg, const GripperReleaseAction gripper_release_action)
{
  switch (gripper_release_action)
  {
    case GripperReleaseAction::NormalRelease:
      set_bits(reg, gATR_mask, 0b00000000);
      break;
    case GripperReleaseAction::ReleaseWithoutTimeout:
      set_bits(reg, gATR_mask, 0b00010000);
      break;
    default:
      break;
  }
}

const std::string gripper_release_action_to_string(const GripperReleaseAction gripper_release_action)
{
  // clang-format off
  static std::map<GripperReleaseAction, std::string> map = {
    { GripperReleaseAction::NormalRelease, "NormalRelease" },
    { GripperReleaseAction::ReleaseWithoutTimeout, "ReleaseWithoutTimeout" } };
  // clang-format on
  return map.at(gripper_release_action);
}

///////////////////////////////////////////////////////////////////////////////
/// Activation status.
///

constexpr uint8_t gSTA_mask = 0b00110000;

GripperActivationStatus get_gripper_activation_status(const uint8_t& reg)
{
  // clang-format off
  static const std::unordered_map<uint8_t, GripperActivationStatus> map{
    { 0b00000000, GripperActivationStatus::GripperNotActivated },
    { 0b00010000, GripperActivationStatus::Unknown },
    { 0b00110000, GripperActivationStatus::GripperOperational },
    { 0b00100000, GripperActivationStatus::Unknown } };
  // clang-format on
  return map.at(reg & default_driver_utils::gSTA_mask);
}

const std::string gripper_activation_status_to_string(const GripperActivationStatus gripper_activation_status)
{
  // clang-format off
  static std::map<GripperActivationStatus, std::string> map = {
    { GripperActivationStatus::GripperNotActivated, "GripperNotActivated" },
    { GripperActivationStatus::GripperOperational, "GripperOperational" },
    { GripperActivationStatus::Unknown, "Unknown" } };
  // clang-format on
  return map.at(gripper_activation_status);
}

///////////////////////////////////////////////////////////////////////////////
/// Object status
///

constexpr uint8_t gOBJ_mask = 0b11000000;

ObjectDetectionStatus get_object_detection_status(const uint8_t& reg)
{
  // clang-format off
  static const std::unordered_map<uint8_t, ObjectDetectionStatus> map{
    { 0b00000000, ObjectDetectionStatus::Unknown },
    { 0b01000000, ObjectDetectionStatus::ObjectDetectedAtMinPressure },
    { 0b10000000, ObjectDetectionStatus::ObjectDetectedAtMaxPressure },
    { 0b11000000, ObjectDetectionStatus::NoObjectDetected } };
  // clang-format on
  return map.at(reg & default_driver_utils::gOBJ_mask);
}

const std::string object_detection_to_string(const ObjectDetectionStatus object_detection)
{
  // clang-format off
  static std::map<ObjectDetectionStatus, std::string> map = {
    { ObjectDetectionStatus::Unknown, "Unknown" },
    { ObjectDetectionStatus::ObjectDetectedAtMinPressure, "ObjectDetectedAtMinPressure" },
    { ObjectDetectionStatus::ObjectDetectedAtMaxPressure, "ObjectDetectedAtMaxPressure" },
    { ObjectDetectionStatus::NoObjectDetected, "NoObjectDetected" } };
  // clang-format on
  return map.at(object_detection);
}

double object_detection_to_double(const ObjectDetectionStatus object_detection)
{
  // clang-format off
  static std::map<ObjectDetectionStatus, double> map = {
    { ObjectDetectionStatus::Unknown, 0.0 },
    { ObjectDetectionStatus::ObjectDetectedAtMinPressure, 1.0 },
    { ObjectDetectionStatus::ObjectDetectedAtMaxPressure, 2.0 },
    { ObjectDetectionStatus::NoObjectDetected, 3.0 } };
  // clang-format on
  return map.at(object_detection);
}

///////////////////////////////////////////////////////////////////////////////
/// Gripper fault status
///

constexpr uint8_t gFLT_mask = 0b00001111;

GripperFaultStatus get_gripper_fault_status(const uint8_t& reg)
{
  // clang-format off
  static const std::unordered_map<uint8_t, GripperFaultStatus> map{
    { 0b00000000, GripperFaultStatus::NoFault },                             // 0x0
    { 0b00000001, GripperFaultStatus::Unknown },                             // 0x1
    { 0b00000010, GripperFaultStatus::Unknown },                             // 0x2
    { 0b00000011, GripperFaultStatus::PorousMaterialDetected },              // 0x3
    { 0b00000100, GripperFaultStatus::Unknown },                             // 0x4
    { 0b00000101, GripperFaultStatus::ActionDelayed },                       // 0x5
    { 0b00000110, GripperFaultStatus::GrippingTimeout },                     // 0x6
    { 0b00000111, GripperFaultStatus::ActivationBitNotSet },                 // 0x7
    { 0b00001000, GripperFaultStatus::MaximumTemperatureExceeded },          // 0x8
    { 0b00001001, GripperFaultStatus::NoCommunicationForAtLeastOneSecond },  // 0x9
    { 0b00001010, GripperFaultStatus::UnderMinimumOperatingVoltage },        // 0xA
    { 0b00001011, GripperFaultStatus::AutomaticReleaseInProgress },          // 0xB
    { 0b00001100, GripperFaultStatus::InternalFault },                       // 0xC
    { 0b00001101, GripperFaultStatus::Unknown },                             // 0xD
    { 0b00001110, GripperFaultStatus::Unknown },                             // 0xE
    { 0b00001111, GripperFaultStatus::AutomaticReleaseCompleted }            // 0xF
  };
  // clang-format on
  return map.at(reg & default_driver_utils::gFLT_mask);
}

const std::string fault_status_to_string(const GripperFaultStatus fault_status)
{
  // clang-format off
  static std::map<GripperFaultStatus, std::string> map = {
    { GripperFaultStatus::NoFault, "NoFault" },
    { GripperFaultStatus::ActionDelayed, "ActionDelayed" },
    { GripperFaultStatus::PorousMaterialDetected, "PorousMaterialDetected" },
    { GripperFaultStatus::GrippingTimeout, "GrippingTimeout" },
    { GripperFaultStatus::ActivationBitNotSet, "ActivationBitNotSet" },
    { GripperFaultStatus::MaximumTemperatureExceeded, "MaximumTemperatureExceeded" },
    { GripperFaultStatus::NoCommunicationForAtLeastOneSecond, "NoCommunicationForAtLeastOneSecond" },
    { GripperFaultStatus::UnderMinimumOperatingVoltage, "UnderMinimumOperatingVoltage" },
    { GripperFaultStatus::AutomaticReleaseInProgress, "AutomaticReleaseInProgress" },
    { GripperFaultStatus::InternalFault, "InternalFault" },
    { GripperFaultStatus::AutomaticReleaseCompleted, "AutomaticReleaseCompleted" },
    { GripperFaultStatus::Unknown, "Unknown" },
    };
  // clang-format on
  return map.at(fault_status);
}

///////////////////////////////////////////////////////////////////////////////
/// Actuator status
///

constexpr uint8_t gVAS_mask = 0b00000011;

ActuatorStatus get_actuator_status(const uint8_t& reg)
{
  // clang-format off
  static const std::unordered_map<uint8_t, ActuatorStatus> map{
    { 0b00, ActuatorStatus::Standby },
    { 0b01, ActuatorStatus::Gripping },
    { 0b10, ActuatorStatus::PassiveReleasing },
    { 0b11, ActuatorStatus::ActiveReleasing } };
  // clang-format on
  return map.at(reg & default_driver_utils::gVAS_mask);
}

const std::string actuator_status_to_string(const ActuatorStatus actuator_status)
{
  // clang-format off
  static std::map<ActuatorStatus, std::string> map = {
    { ActuatorStatus::Standby, "Standby" },
    { ActuatorStatus::Gripping, "Gripping" },
    { ActuatorStatus::PassiveReleasing, "PassiveReleasing" },
    { ActuatorStatus::ActiveReleasing, "ActiveReleasing" } };
  // clang-format on
  return map.at(actuator_status);
}

}  // namespace epick_driver::default_driver_utils
