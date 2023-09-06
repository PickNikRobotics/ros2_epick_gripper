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

#include <string>

#include <epick_driver/driver.hpp>

/**
 * In this utility class we have methods to:
 * - set and get information in and out of a byte register using enums;
 * - converts enums into strings for testing and debugging.
 */
namespace epick_driver::default_driver_utils
{

/** These represent read and write actions. */
enum class FunctionCode : uint8_t
{
  ReadInputRegisters = 0x04,
  PresetSingleRegister = 0x06,
  PresetMultipleRegisters = 0x10,
  MasterReadWriteMultipleRegisters = 0x17,
};

///////////////////////////////////////////////////////////////////////////////
/// Gripper activation request.
///

/**
 * Set the gripper activation action bits into the given register.
 * @param reg The register to update.
 * @param gripper_activation_action An enum indicaing the bits to be set.
 */
void set_gripper_activation_action(uint8_t& reg, const GripperActivationAction gripper_activation_action);

GripperActivationAction get_gripper_activation_action(const uint8_t& reg);

/**
 * Convert a GripperActivationAction enum into a string.
 * @param gripper_activation_action The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_activation_action_to_string(const GripperActivationAction gripper_activation_action);

///////////////////////////////////////////////////////////////////////////////
/// Gripper mode.
///

/**
 * Set the gripper mode bits into the given register.
 * @param reg The register to update.
 * @param gripper_mode An enum indicaing the bits to be set.
 */
void set_gripper_mode(uint8_t& reg, const GripperMode gripper_mode);

/**
 * Get the gripper mode from the given register by reading the corresponding bits.
 * @param reg The register to read.
 * @param The gripper mode.
 */
GripperMode get_gripper_mode(const uint8_t& reg);

/**
 * Convert a GripperMode enum into a string.
 * @param gripper_mode The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_mode_to_string(const GripperMode gripper_mode);

///////////////////////////////////////////////////////////////////////////////
/// Regulate.
///

/**
 * Set the gripper regulate action bits into the given register.
 * @param reg The register to update.
 * @param regulate_action An enum indicaing the bits to be set.
 */
void set_gripper_regulate_action(uint8_t& reg, const GripperRegulateAction regulate_action);

/**
 * Get the gripper regulate action from the given register by reading the corresponding bits.
 * @param reg The register to read.
 * @param The gripper regulate action.
 */
GripperRegulateAction get_gripper_regulate_action(const uint8_t& reg);

/**
 * Convert a GripperRegulateAction enum into a string.
 * @param gripper_regulate_action The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_regulate_action_to_string(const GripperRegulateAction gripper_regulate_action);

/**
 * All Command and State Interfaces are double. We use this function to convert a double into
 * an enum.
 * @return A double to be converted into an enum.
 */
GripperRegulateAction double_to_regulate_action(double regulate);

/**
 * All Command and State Interfaces are double. We use this function to convert an enum into
 * a adouble.
 * @return The enum representing the given double.
 */
double regulate_action_to_double(GripperRegulateAction regulate);

///////////////////////////////////////////////////////////////////////////////
/// Automatic release action.
///

/**
 * Set the gripper automatic release action bits into the given register.
 * @param reg The register to update.
 * @param gripper_release_action An enum indicaing the bits to be set.
 */
void set_gripper_automatic_release_action(uint8_t& reg, const GripperReleaseAction gripper_release_action);

/**
 * Convert a GripperReleaseAction enum into a string.
 * @param gripper_release_action The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_release_action_to_string(const GripperReleaseAction gripper_release_action);

///////////////////////////////////////////////////////////////////////////////
/// Activation status.
///

/**
 * Get the gripper activations status from the given register by reading the corresponding bits.
 * @param reg The register to read.
 * @param The gripper activations status.
 */
GripperActivationStatus get_gripper_activation_status(const uint8_t& reg);

/**
 * Convert a GripperActivationStatus enum into a string.
 * @param gripper_activation_status The enum.
 * @return A string representation of the given enum
 */
const std::string gripper_activation_status_to_string(const GripperActivationStatus gripper_activation_status);

///////////////////////////////////////////////////////////////////////////////
/// Object status
///

/**
 * Get the object detection status from the given register by reading the corresponding bits.
 * @param reg The register to read.
 * @param The object detection status.
 */
ObjectDetectionStatus get_object_detection_status(const uint8_t& reg);

/**
 * Convert an ObjectDetection enum into a string.
 * @param object_detection The enum.
 * @return A string representation of the given enum
 */
const std::string object_detection_to_string(const ObjectDetectionStatus object_detection);

/**
 * All Command and State Interface are double. We use this function to convert an enum into
 * a double representation, so that we can later recover it back.
 * @return A double representing the enum.
 */
double object_detection_to_double(const ObjectDetectionStatus object_detection);

///////////////////////////////////////////////////////////////////////////////
/// Gripper fault status
///

/**
 * Get the gripper fault status from the given register by reading the corresponding bits.
 * @param reg The register to read.
 * @param The gripper fault status.
 */
GripperFaultStatus get_gripper_fault_status(const uint8_t& reg);

/**
 * Convert a FaultStatus enum into a string.
 * @param fault_status The enum.
 * @return A string representation of the given enum
 */
const std::string fault_status_to_string(const GripperFaultStatus fault_status);

///////////////////////////////////////////////////////////////////////////////
/// Actuator status
///

/**
 * Get the gripper actuator status from the given register by reading the corresponding bits.
 * @param reg The register to read.
 * @param The gripper actuator status.
 */
ActuatorStatus get_actuator_status(const uint8_t& reg);

/**
 * Convert a ActuatorStatus enum into a string.
 * @param actuator_status The enum.
 * @return A string representation of the given enum
 */
const std::string actuator_status_to_string(const ActuatorStatus actuator_status);

};  // namespace epick_driver::default_driver_utils
