#pragma once

#include <unitree_legged_sdk/comm.h>

#include "control_communicator.hpp"


namespace unitree_go1_bridge::utility
{
//! @param [in, out] command
void zeroResetLowCommand(ControlCommunicator::Command &command);

void resetMotorCommand(ControlCommunicator::MotorCommand &motor_command);
}  // namespace unitree_go1_bridge::utility
