#include <unitree_go1_bridge/utility.hpp>


namespace unitree_go1_bridge::utility
{
namespace unitree_legged_sdk = UNITREE_LEGGED_SDK;

void zeroResetLowCommand(ControlCommunicator::Command &command)
{
  for(auto &&head : command.head)
  {
    head = 0;
  }
  command.levelFlag = 0;
  command.frameReserve = 0;
  for(auto &&sn : command.SN)
  {
    sn = 0;
  }
  for(auto &&version : command.version)
  {
    version = 0;
  }
}

void resetMotorCommand(ControlCommunicator::MotorCommand &motor_command)
{
  motor_command.mode = 0;
  motor_command.q = unitree_legged_sdk::PosStopF;
  motor_command.dq = unitree_legged_sdk::VelStopF;
  motor_command.tau = 0;
  motor_command.Kp = 0;
  motor_command.Kd = 0;
}
}  // namespace unitree_go1_bridge::utility
