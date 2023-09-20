#ifndef _H_DEFINE
#define _H_DEFINE

#if defined(__SIMU__) & defined(ARDUINO)
#warning trying to use simu node on a real teensy card
#endif

#ifndef __linux__
#include <Arduino.h>
#undef abs
#elif defined(__SIMU__)
#include "../Simulation/Arduino_defines.h"
typedef bool boolean;
#endif
//code de l'asserv
#define FILTRE_ANGLE // ajoute un filtre lineaire sur la position en Theta

#define ODO_HARD // utilise le decodeur hardware pour les odometres positions

#include <cmath>

//Macros
#define sign(value) (value > 0 ? 1 : -1)

// #define ASSERV_MOTEURS

#define ODRIVE_RX_PIN 0
#define ODRIVE_TX_PIN 1

#define MAX_MOTOR_SPEED 8 // turns/s


/* TRAJECTOIRES */

#define MAX_LINEAR_GOAL_SPEED 0.22 // m/s //TODO set
#define MAX_ROTATION_GOAL_SPEED 1.57 // rad/s //TODO set

#define DEFAULT_LINEAR_ACCEL_PARAM 0.2 // m/s^2 //TODO set
#define DEFAULT_ROTATION_ACCEL_PARAM 3.0 // rad/s^2 //TODO set

// longueur de trajectoire en-dessous de laquelle on ignore la trajectore
#define EPSILON_DTOTALE 0.001 // m


/* RECALAGES */
#define RECAL_ASSERV_TIMEOUT 10.0 // s //TODO set
#define RECAL_SPEED 0.1 // m/s

// distance below which we command the motors directly
#define RECAL_DISTANCE 0.1 // m //TODO set


#endif
