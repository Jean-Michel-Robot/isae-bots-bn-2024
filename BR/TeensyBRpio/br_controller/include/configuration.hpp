#ifndef _CONFIGURATION_HPP_
#define _CONFIGURATION_HPP_

/* PID */
// See classes Integral, Derivative and ProportionalIntegralDerivative (folder math)

#define DEFAULT_KP 13.2
#define DEFAULT_TI 0.25
#define DEFAULT_TD 0.167
#define DERIVATIVE_FILTER 5.0

// Use {} instead of a number to disable saturation
#define INTEGRAL_SATURATION 0.5
#define DERIVATIVE_SATURATION 10.0
#define PID_SATURATION 10.0

/* CONTROLLER */
// See controller/UnicycleController.hpp and manager/ControllerManager.hpp

// x offset of the tracking point (should be strictly greater than 0)
#define ASSERV_ALPHA 0.10 // m
// y offset of the the tracking point (can be set to 0 if the point is centered)
#define ASSERV_BETA 0.0 // m

// TODO: measure the acceptable tick rate on Arduino.
#define UPDATE_INTERVAL 200 // µS (NOT milliseconds)

/* TRAJECTORIES */
// TODO check the speeds and accelerations on the real robot

#define MAX_LINEAR_GOAL_SPEED 0.5   // m/s
#define MAX_ROTATION_GOAL_SPEED 3.0 // rad/s

#define DEFAULT_LINEAR_ACCELERATION 0.1   // m/s^2
#define DEFAULT_ROTATION_ACCELERATION 3.0 // rad/s^2

#define BRAKING_LINEAR_ACCELERATION 1.5   // m/s^2, >= DEFAULT_LINEAR_ACCELERATION
#define BRAKING_ROTATION_ACCELERATION 3.0 // rad/s^2, >= DEFAULT_ROTATION_ACCELERATION

/* ROS */

#ifdef _SIMULATION
#define SEND_POSITION_INTERVAL 10 // ms (for a smooth GUI)
#else
// Also applies to topic /odos_count
#define SEND_POSITION_INTERVAL 100 // ms
#endif

// Applies to topic /logTotaleArray
#define ROS_LOG_INTERVAL 100 // ms

/* ODOS (See feedback/PositionEstimatorOdo.hpp) - Not used by simulation */

#define ODOS_METHOD MethodMoveFirst

// Calibrated values
#define ECARTS_ODOS 5980.73537126610L // (ticks.rad^(-1) ecart entre les 2 odos
#define UNITS_ODOS 51.54179961710274L // ticks.mm^(-1)
#define L_R_ODOS 1.0011809854125424L  // Correction factor between the encoders

/* WHEELS - Not used by simulation*/

#define WHEEL_DISTANCE 0.22  // m
#define WHEEL_DIAMETER 0.06  // m
#define TRANSMISSION_RATIO 1 // reduction factor
#define MAX_MOTOR_SPEED 8    // turns/s

/* PINS - Not used by simulation */

#define ODRIVE_RX_PIN 0
#define ODRIVE_TX_PIN 1
// The pin number and blinking interval of the LED (Arduino only) are defined in lib/Led/Led.cpp

/* SIMULATION - Not used on the Teensy */

// To test robustness
#define NOISE_STD_DEV 0.1 // TODO improve the way the noise is implemented

#endif