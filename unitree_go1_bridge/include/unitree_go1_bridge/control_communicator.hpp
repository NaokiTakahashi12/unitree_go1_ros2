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

#pragma once

#include <memory>
#include <string>

#define BOOST_BIND_NO_PLACEHOLDERS
#include <unitree_legged_sdk/unitree_legged_sdk.h>


namespace unitree_go1_bridge
{
namespace unitree_legged_sdk = UNITREE_LEGGED_SDK;

class ControlCommunicator
{
public:
  using Command = unitree_legged_sdk::LowCmd;
  using MotorCommand = unitree_legged_sdk::MotorCmd;
  using State = unitree_legged_sdk::LowState;

  ControlCommunicator();
  ~ControlCommunicator();

  void setMotorCommand(const MotorCommand &, const unsigned int motor_index);

  const State getLatestState();

  void send();

  void receive(State &);
  const State receive();

private:
  uint8_t m_control_level;
  uint16_t m_local_port;
  uint16_t m_target_port;

  std::string m_target_ip_address;

  std::unique_ptr<unitree_legged_sdk::UDP> m_unitree_udp;
  std::unique_ptr<unitree_legged_sdk::Safety> m_unitree_safety;

  std::unique_ptr<State> m_state;
  std::unique_ptr<Command> m_command;

  void ignoreScreenOut();
  void enableScreenOut();
};
}  // namespace unitree_go1_bridge
