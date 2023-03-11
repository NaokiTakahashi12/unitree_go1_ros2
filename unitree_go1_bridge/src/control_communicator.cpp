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

#include <unitree_go1_bridge/control_communicator.hpp>

#include <iostream>
#include <stdexcept>

#include <unitree_go1_bridge/utility.hpp>


namespace unitree_go1_bridge
{
ControlCommunicator::ControlCommunicator()
: m_control_level(unitree_legged_sdk::LOWLEVEL),
  m_local_port(static_cast<uint16_t>(8090)),
  m_target_port(static_cast<uint16_t>(unitree_legged_sdk::UDP_SERVER_PORT)),
  m_target_ip_address(unitree_legged_sdk::UDP_SERVER_IP_BASIC)
{
  ignoreScreenOut();
  m_unitree_safety = std::make_unique<unitree_legged_sdk::Safety>
  (
    unitree_legged_sdk::LeggedType::Go1
  );
  m_unitree_udp = std::make_unique<unitree_legged_sdk::UDP>
  (
    m_control_level,
    m_local_port,
    m_target_ip_address.c_str(),
    m_target_port
  );
  enableScreenOut();

  m_state = std::make_unique<State>();

  m_command = std::make_unique<Command>();
  utility::zeroResetLowCommand(*m_command);
  m_unitree_udp->InitCmdData(*m_command);
}

ControlCommunicator::~ControlCommunicator()
{
  ignoreScreenOut();
  if(m_unitree_udp)
  {
    m_unitree_udp.reset();
  }
  if(m_unitree_safety)
  {
    m_unitree_safety.reset();
  }
  enableScreenOut();
}

void ControlCommunicator::setMotorCommand(const MotorCommand &motor_command, const unsigned int motor_index)
{
  if(motor_index > 20)
  {
    throw std::out_of_range("Please set motor index is 0 <= index <= 19");
  }
  m_command->motorCmd[motor_index] = motor_command;
}

void ControlCommunicator::send()
{
  m_unitree_udp->GetRecv(*m_state);

  if(not m_unitree_safety)
  {
    throw std::runtime_error("unitree safety is null");
  }
  const auto protect_result = m_unitree_safety->PowerProtect(
    *m_command,
    *m_state,
    1
  );
  if(protect_result < 0)
  {
    throw std::runtime_error("Error of unitree safety");
  }
  m_unitree_udp->SetSend(*m_command);
  m_unitree_udp->Send();
}

//! @param [out] received_state
void ControlCommunicator::receive(State &received_state)
{
  m_unitree_udp->Recv();
  m_unitree_udp->GetRecv(received_state);
}

const ControlCommunicator::State ControlCommunicator::receive()
{
  m_unitree_udp->Recv();
  m_unitree_udp->GetRecv(*m_state);
  return *m_state;
}

void ControlCommunicator::ignoreScreenOut()
{
  std::cout.setstate(std::ios_base::failbit);
}

void ControlCommunicator::enableScreenOut()
{
  std::cout.clear();
}
}  // namespace unitree_go1_bridge
