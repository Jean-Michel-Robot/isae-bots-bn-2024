/*
  Handles communication with the cytron card
*/

#include "CytronMotor.h"
#include "a_extern.h"
#include <cmath>

// The command is in sign magnitude mode : 
// PWM pin controls the speed, value between 0 & 255
// DIR pin controls direction (LOW CCW, HIGH, CW)

// constructor of CytronMotor class
CytronMotor::CytronMotor(int id, int motor_number int minCommand, int maxCommand) {
  this->id = id;
  this->motor_number = motor_number;
  this->minCommand = minCommand;
  this ->maxCommand = maxCommand;
}

// initialize the motor
void CytronMotor::initMotor() {
  pinMode(pwmPin, OUTPUT); // pwm pin as output
  analogWriteFrequency(pwmPin, 31000);
  pinMode(dirPin, OUTPUT); // dir pin as output  
  digitalWrite(dirPin, HIGH); // CW by default
  analogWrite(pwmPin, 0); // motor stopped
}

// stops the motor
void CytronMotor::stopMotor() {
    analogWrite(pwmPin, 0); // motor stopped
}

// send command to the motor
void CytronMotor::commandMotor(int value) {
	// static bool dirCW = true;

	// // checks if the value sent to the motor is within range, and sets the direction
	// if (value > 0) {
	// 	dirCW = true;
	// 	value = value > maxCommand ? maxCommand : (value < minCommand ? minCommand : value);
	// }
	// else if (value < 0) {
	// 	dirCW = false;
	// 	value *= -1;
	// 	value = value > maxCommand ? maxCommand : (value < minCommand ? minCommand : value);
	// }
	// else value = 0;

  // digitalWrite(dirPin, dirCW); // writes dir value to dir pin
	// analogWrite(pwmPin, value); // writes speed to the pwm pin in order to control the motor's speed
  // // Serial.println(String(dirCW) + " " + String(value));

      //TODO : transform velCmd into odrive command (nb_turn/s)
    // knowing the wheel diameter and the transmission ratio
    float odrv_cmd = value*TRANSMISSION_RATIO/(PI*WHEEL_DIAMETER);

    // constrain the motor command for safety
    // if (abs(odrv_cmd) > 10) {
    //     p_ros->logPrint(ERROR, "Valeur de commande Odrive supérieure au seuil");
    // }
    odrv_cmd = constrain(odrv_cmd, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);


    // send value or its opposite because motors are symmetrical
    if (motor_number == BR_LEFT) {odrive.SetVelocity(motor_number, -odrv_cmd);}
    else {odrive.SetVelocity(motor_number, odrv_cmd);}
}
