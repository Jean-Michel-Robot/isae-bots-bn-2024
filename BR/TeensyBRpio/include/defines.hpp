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

#endif

