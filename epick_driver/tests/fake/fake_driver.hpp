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

#include <epick_driver/default_driver.hpp>

namespace epick_driver
{
class FakeDriver : public Driver
{
public:
  void set_slave_address(const uint8_t slave_address) override
  {
    slave_address_ = slave_address;
  }

  void set_mode(const GripperMode gripper_mode) override
  {
    gripper_mode_ = gripper_mode;
  }

  void set_max_vacuum_pressure(const float vacuum_pressure) override
  {
    max_vacuum_pressure_ = vacuum_pressure;
  }

  void set_min_vacuum_pressure(const float vacuum_pressure) override
  {
    min_vacuum_pressure_ = vacuum_pressure;
  }

  void set_gripper_timeout(const std::chrono::milliseconds timeout) override
  {
    gripper_timeout_ = timeout;
  }

  bool connect() override
  {
    connected_ = true;
    return true;
  }

  void disconnect() override
  {
    connected_ = false;
  }

  void activate() override
  {
    activated_ = true;
  }

  void deactivate() override
  {
    activated_ = false;
  }

  void grip() override
  {
    regulate_ = true;
  }

  void release() override
  {
    regulate_ = false;
  }

  GripperStatus get_status() override
  {
    GripperStatus status;
    status.gripper_activation_action =
        activated_ ? GripperActivationAction::Activate : GripperActivationAction::ClearGripperFaultStatus;
    status.gripper_mode = gripper_mode_;
    status.gripper_regulate_action =
        regulate_ ? GripperRegulateAction::FollowRequestedVacuumParameters : GripperRegulateAction::StopVacuumGenerator;
    status.gripper_activation_status =
        activated_ ? GripperActivationStatus::GripperOperational : GripperActivationStatus::GripperNotActivated;
    status.gripper_fault_status = GripperFaultStatus::NoFault;
    status.actuator_status = regulate_ ? ActuatorStatus::Gripping : ActuatorStatus::PassiveReleasing;
    status.object_detection_status =
        regulate_ ? ObjectDetectionStatus::ObjectDetectedAtMaxPressure : ObjectDetectionStatus::NoObjectDetected;
    status.max_vacuum_pressure = max_vacuum_pressure_;
    status.actual_vacuum_pressure = (max_vacuum_pressure_ + min_vacuum_pressure_) / 2;
    return status;
  }

private:
  uint8_t slave_address_ = 0x00;
  GripperMode gripper_mode_ = GripperMode::Unknown;
  float max_vacuum_pressure_ = 0.0f;
  float min_vacuum_pressure_ = 0.0f;
  std::chrono::milliseconds gripper_timeout_ = std::chrono::milliseconds(100);
  bool connected_ = false;
  bool activated_ = false;
  bool regulate_ = false;
};

}  // namespace epick_driver
