#ifndef _DEFINES_HPP
#define _DEFINES_HPP

/* ODOS*/

#define ODOS_METHOD 1
#define ODO_HARD


/* ASSERV */
#define WHEEL_DISTANCE 0.25 //m


// Macros
#define sign(value) (value > 0 ? 1 : -1)

// Parameters
#define ODRIVE_RX_PIN 0
#define ODRIVE_TX_PIN 1

#define ODO_SEND_POSITION_TIMER 100 //ms

#define OBJECTIVE_THRESHOLD_X      0.005 //m
#define OBJECTIVE_THRESHOLD_Y      0.005 //m
#define OBJECTIVE_THRESHOLD_THETA  0.008 //rad (0.008 rad ~ .5 deg)

#define WHEEL_DIAMETER 0.07 // m
#define TRANSMISSION_RATIO 1 // reduction factor

#define DEFAULT_LINEAR_GOAL_SPEED 1.0 // m/s //TODO set
#define DEFAULT_ROTATION_GOAL_SPEED 1.0 // m/s //TODO set

#define DEFAULT_LINEAR_ACCEL_PARAM 0.5 // m/s^2 //TODO set
#define DEFAULT_ROTATION_ACCEL_PARAM 5 // m/s^2 //TODO set



// Asserv

// x coord in robot frame of the tracking point (should be strictly greater than 0)
#define ASSERV_ALPHA 0.10 // m //TODO set

// y coord in robot frame of the tracking point (can be set to 0 if the point is centered)
#define ASSERV_BETA 0.0 // m //TODO set

#endif

