#ifdef __SIMU__

#include <algorithm>
#include "motors.hpp"

#include "feedback/StateSimulator.hpp"


int leftMotorState = AxisState::AXIS_STATE_IDLE;
int rightMotorState = AxisState::AXIS_STATE_IDLE;

void motors_init() {
    leftMotorState = AxisState::AXIS_STATE_IDLE;
    rightMotorState = AxisState::AXIS_STATE_IDLE;
}

void sendMotorCommand(int motor_number, float velCmd)
{
    velCmd = std::clamp(velCmd, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    if (motor_number == BR_LEFT)
    {
        StateSimulator::instance().setLeftMotorSpeed(velCmd);
    }
    else if (motor_number == BR_RIGHT)
    {
        StateSimulator::instance().setRightMotorSpeed(velCmd);
    }
}

void setMotorsToIdle()
{
    leftMotorState = AxisState::AXIS_STATE_IDLE;
    rightMotorState = AxisState::AXIS_STATE_IDLE;
}
void setMotorsToClosedLoop()
{
    leftMotorState = AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
    rightMotorState = AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
}

void getCurrentMotorStates(int *states)
{
    states[0] = leftMotorState;
    states[1] = rightMotorState;
}

#endif