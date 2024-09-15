#ifndef _H_MOTORS
#define _H_MOTORS

#include "geometry/Position2D.h"

// defines to avoid wondering if 0 is left or right
#define BR_RIGHT 0
#define BR_LEFT 1

#ifdef ARDUINO 
#include "OdriveArduino/ODriveEnums.h"
#else
// Note that the values do not have to match those of Arduino
enum AxisState {
    AXIS_STATE_IDLE                          = 1,
    AXIS_STATE_CLOSED_LOOP_CONTROL           = 8,
};
#endif

// TODO make more object-oriented
// This is hard because tinyfsm's states are static

// Defines the interface with the robot.
// The state machine does not have to know how those methods are actually implemented.
// They can either delegate to the actual robot or to a simulation.

void motors_init();
void sendMotorCommand(int motor_number, float velCmd);

void setMotorsToIdle();
void setMotorsToClosedLoop();

void getCurrentMotorStates(int* states);

#endif