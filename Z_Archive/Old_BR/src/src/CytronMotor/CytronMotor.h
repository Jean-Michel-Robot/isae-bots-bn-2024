/*
  Handles communication with the cytron card
	Allows for the creation of motor objects
*/

#ifndef __CYTRONMOTORS_H
#define __CYTRONMOTORS_H
#ifndef __linux__
#include <Arduino.h>
#else
#include "../../../Simulation/Arduino_defines.h"
#endif

class CytronMotor {
  private:
    int id; // id of the motor (not very useful in most cases)
    int pwmPin; // pwm pin associated to the motor
    int dirPin; // direction pin associated to the motor
		int minCommand;	// minimum command that can be sent to the motor (0 excluded)
		int maxCommand; // maximum command that can be sent to the motor (0 excluded)
    int motor_number;

  public:
    CytronMotor(int id, int pwmPin, int dirPin, int minCommand, int maxCommand); // constructor of CytronMotor class

    void initMotor(); // initializes the motor

    void stopMotor(); // stops the motor

    void commandMotor(int value); // sends command to the motor
};

#endif
