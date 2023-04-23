#ifndef _MOTORS
#define _MOTORS

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

#include "defines.hpp"


template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


typedef struct motor_inputBloc {
    float cmd_ML;
    float cmd_MR;
} motor_inputBloc;

typedef struct motor_contextBloc {

} motor_contextBloc;

// typedef struct motor_outputBloc {

// } motor_outputBloc;


// Functions
void motors_init();
void sendMotorCommand(int motor_number, float velCmd);

#endif