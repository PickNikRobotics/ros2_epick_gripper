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

#include <chrono>
#include <string>

namespace epick_driver
{

// Indicates if the user has requested the gripper to be activated.
enum class GripperActivationAction
{
  ClearGripperFaultStatus,
  Activate,
};

// Indicates the gripper mode.
enum class GripperMode
{
  AutomaticMode,  // Automatic.
  AdvancedMode,   // Advanced.
  Unknown
};

enum class GripperRegulateAction
{
  StopVacuumGenerator,             // Stop the vacuum generator.
  FollowRequestedVacuumParameters  // Follow the requested vacuum parameters in real time.
};

enum class GripperReleaseAction
{
  NormalRelease,         // Normal operation.
  ReleaseWithoutTimeout  // Open the valves without any timeout.
};

// Indicates the status of the gripper activation sequence.
enum class GripperActivationStatus
{
  GripperNotActivated,  // Gripper is not activated.
  GripperOperational,   // Gripper is operational.
  Unknown
};

// Indicates the status of the object detection.
enum class ObjectDetectionStatus
{
  ObjectDetectedAtMinPressure,  // Object detected. Minimum vacuum value reached.
  ObjectDetectedAtMaxPressure,  // Object detected. Maximum vacuum value reached.
  NoObjectDetected,             // Object loss, dropped or gripping timeout reached.
  Unknown                       // Unknown object detection. Regulating towards requested vacuum/pressure.
};

enum class GripperFaultStatus
{
  NoFault,
  ActionDelayed,
  PorousMaterialDetected,
  GrippingTimeout,
  ActivationBitNotSet,
  MaximumTemperatureExceeded,
  NoCommunicationForAtLeastOneSecond,
  UnderMinimumOperatingVoltage,
  AutomaticReleaseInProgress,
  InternalFault,
  AutomaticReleaseCompleted,
  Unknown
};

enum class ActuatorStatus
{
  Standby,
  Gripping,
  PassiveReleasing,
  ActiveReleasing
};

struct GripperStatus
{
  GripperActivationAction gripper_activation_action;
  GripperMode gripper_mode;
  GripperRegulateAction gripper_regulate_action;
  GripperActivationStatus gripper_activation_status;
  GripperFaultStatus gripper_fault_status;
  ActuatorStatus actuator_status;
  ObjectDetectionStatus object_detection_status;
  float max_vacuum_pressure;
  float actual_vacuum_pressure;
};

/**
 * This is the interface of the driver to control the Epick Gripper.
 * The Driver interface can be easily mocked for testing or implemented to
 * fake the behavior of the real hardware.
 */
class Driver
{
public:
  Driver() = default;

  virtual void set_slave_address(uint8_t slave_address) = 0;

  virtual void set_mode(GripperMode gripper_mode) = 0;

  /**
   * Set the gripper maximum vacuum pressure to hold an object relative to the
   * atmospheric pressure.
   * The vacuum pressure is measured in kPa below the atmospheric pressure
   * (100KpA) and must fall between -100kPa (perfect vacuum) and 0kPa.
   * @param vacuum_pressure The maximum grip vacuum pressure between
   * -100kPa and 0kPa.
   */
  virtual void set_grip_max_vacuum_pressure(float vacuum_pressure) = 0;

  /**
   * Set the gripper minimum acceptable vacuum pressure to hold an object
   * relative to the atmospheric pressure.
   * The vacuum pressure is measured in kPa below the atmospheric pressure
   * (100KpA) and must fall between -100kPa (perfect vacuum) and 0kPa.
   * @param vacuum_pressure The minimum acceptable vacuum pressure between
   * -100kPa and 0kPa.
   */
  virtual void set_grip_min_vacuum_pressure(float vacuum_pressure) = 0;

  virtual void set_grip_timeout(std::chrono::milliseconds timeout) = 0;

  virtual void set_release_timeout(std::chrono::milliseconds timeout) = 0;

  /** Connect to the gripper serial connection. */
  virtual bool connect() = 0;

  /** Disconnect from the gripper serial connection. */
  virtual void disconnect() = 0;

  virtual void activate() = 0;
  virtual void deactivate() = 0;

  virtual void grip() = 0;
  virtual void release() = 0;

  virtual GripperStatus get_status() = 0;
};
}  // namespace epick_driver
