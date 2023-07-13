// BSD 3-Clause License
//
// Copyright (c) 2023, NaokiTakahashi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <unitree_go1_bridge/utility.hpp>


namespace unitree_go1_bridge::utility
{
void resetMotorCommand(
  ControlCommunicator<unitree_legged_sdk::LOWLEVEL>::MotorCommand & motor_command)
{
  motor_command.mode = 0;
  motor_command.q = unitree_legged_sdk::PosStopF;
  motor_command.dq = unitree_legged_sdk::VelStopF;
  motor_command.tau = 0;
  motor_command.Kp = 0;
  motor_command.Kd = 0;
}

void zeroResetLowCommand(
  ControlCommunicator<unitree_legged_sdk::LOWLEVEL>::Command & command)
{
  for (auto && head : command.head) {
    head = 0;
  }
  command.levelFlag = 0;
  command.frameReserve = 0;
  for (auto && sn : command.SN) {
    sn = 0;
  }
  for (auto && version : command.version) {
    version = 0;
  }
}

void zeroResetHighCommand(
  ControlCommunicator<unitree_legged_sdk::HIGHLEVEL>::Command & command)
{
  for (auto && head : command.head) {
    head = 0;
  }
  command.levelFlag = 0;
  command.frameReserve = 0;
  for (auto && sn : command.SN) {
    sn = 0;
  }
  for (auto && version : command.version) {
    version = 0;
  }
  command.mode = 0;
  command.gaitType = 0;
  command.speedLevel = 0;
  command.footRaiseHeight = 0.0;
  command.bodyHeight = 0.0;
  for (auto && position : command.position) {
    position = 0.0;
  }
  for (auto && euler_angle : command.euler) {
    euler_angle = 0.0;
  }
  for (auto && velocity : command.velocity) {
    velocity = 0.0;
  }
  command.yawSpeed = 0.0;
}

}  // namespace unitree_go1_bridge::utility
