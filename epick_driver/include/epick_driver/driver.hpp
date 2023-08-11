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

namespace epick_driver
{

// Indicates if the user has requested the gripper to be activated.
enum class GripperActivationAction
{
  Disable,
  Activate,
};

// Indicates the gripper mode.
enum class GripperMode
{
  AutomaticMode,  // Automatic.
  AdvancedMode,   // Advanced.
  Reserved
};

enum class GripperRegulateAction
{
  StopVacuumGenerator,             // Stop the vacuum generator.
  FollowRequestedVacuumParameters  // Follow the requested vacuum parameters in real time
};

// Indicates the status of the gripper activation sequence.
enum class GripperActivationStatus
{
  GripperNotActivated,  // Gripper is not activated.
  GripperOperational,   // Gripper is operational.
};

// Indicates the status of the object detection.
enum class ObjectDetectionStatus
{
  Unknown,                      // Unknown object detection. Regulating towards requested vacuum/pressure.
  ObjectDetectedAtMinPressure,  // Object detected. Minimum vacuum value reached.
  ObjectDetectedAtMaxPressure,  // Object detected. Maximum vacuum value reached.
  NoObjectDetected              // Object loss, dropped or gripping timeout reached.
};

enum class GripperFaultStatus
{
  NoFault,
  AcionDelayed,
  PorousMaterialDetected,
  GrippingTimeout,
  ActivationBitNotSet,
  MaximumTemperatureExceeded,
  NoCommunicationForAtLeastOneSecond,
  UderMinimumOperatingVoltage,
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
  float max_pressure_request;
  float actual_pressure;
};

class Driver
{
public:
  Driver() = default;

  virtual bool connect() = 0;

  virtual void disconnect() = 0;

  virtual void activate() = 0;
  virtual void deactivate() = 0;

  virtual void grip() = 0;
  virtual void release() = 0;

  virtual void set_relative_pressure(const float& relative_pressure_kPa) = 0;

  virtual void set_mode() = 0;

  virtual void set_max_device_vacuum() = 0;
  virtual void set_min_device_vacuum() = 0;

  virtual void set_grip_timeout() = 0;
  virtual void set_release_time() = 0;

  virtual GripperStatus get_status() = 0;
};
}  // namespace epick_driver
