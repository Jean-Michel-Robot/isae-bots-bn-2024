#ifndef _MOTORS
#define _MOTORS

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

#include "defines.hpp"


template<typename T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


// Functions
void motors_init();
void sendMotorCommand(int motor_number, float velCmd);

void setMotorsToIdle();
void setMotorsToClosedLoop();

void getCurrentMotorStates(int* states);

#endif