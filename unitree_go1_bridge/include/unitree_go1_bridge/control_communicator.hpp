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
