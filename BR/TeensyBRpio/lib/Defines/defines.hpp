#ifndef _DEFINES_HPP
#define _DEFINES_HPP

// Macros
#define sign(value) (value > 0 ? 1 : -1)


// defines to avoid wondering if 0 is left or right
#define BR_RIGHT 0
#define BR_LEFT 1

/* ODOS*/

#define ODOS_METHOD 1
#define ODO_HARD
#define ODO_SEND_POSITION_TIMER 100 //ms



/* ASSERV */
#define WHEEL_DISTANCE 0.25 //m

// x offset of the tracking point (should be strictly greater than 0)
#define ASSERV_ALPHA 0.15 // m //TODO set to the center of the cakes

// y offset of the the tracking point (can be set to 0 if the point is centered)
#define ASSERV_BETA 0.0 // m //TODO set

#define DEFAULT_OBJECTIVE_THRESHOLD_X      0.005 //m
#define DEFAULT_OBJECTIVE_THRESHOLD_Y      0.005 //m
#define DEFAULT_OBJECTIVE_THRESHOLD_THETA  0.008 //rad (0.008 rad ~ .5 deg)

#define ASSERV_BYPASSED 1  // to use if we want to bypass the asserv for one update


/* COMMANDES MOTEUR */

#define WHEEL_DIAMETER 0.06 // m
#define TRANSMISSION_RATIO 1 // reduction factor

#define MAX_MOTOR_SPEED 8 // turns/s


/* TRAJECTOIRES */

#define DEFAULT_LINEAR_GOAL_SPEED 0.1 // m/s //TODO set
#define DEFAULT_ROTATION_GOAL_SPEED 0.1 // m/s //TODO set

#define DEFAULT_LINEAR_ACCEL_PARAM 0.2 // m/s^2 //TODO set
#define DEFAULT_ROTATION_ACCEL_PARAM 0.2 // m/s^2 //TODO set

// longueur de trajectoire en-dessous de laquelle on ignore la trajectore
#define EPSILON_DTOTALE 0.001 // m


/* RECALAGES */
#define RECAL_ASSERV_TIMEOUT 10.0 // s //TODO set
#define RECAL_SPEED 0.1 // m/s

// distance below which we command the motors directly
#define RECAL_DISTANCE 0.1 // m //TODO set

/* PINS */
#define ODRIVE_RX_PIN 0
#define ODRIVE_TX_PIN 1

#define BUMPER_RIGHT_PIN 15 //TODO set
#define BUMPER_LEFT_PIN 16 //TODO set


/* LOG */
#define LOG_PERIOD 100 // ms

#endif

