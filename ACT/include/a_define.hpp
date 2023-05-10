#ifndef DEFINES_HPP
#define DEFINES_HPP

/* POSITIONS */
//TODO

#define DOORS_LEFT_OPEN_POS    90
#define DOORS_LEFT_CLOSED_POS  165
#define DOORS_RIGHT_OPEN_POS   65
#define DOORS_RIGHT_CLOSED_POS 10


#define ELEV_STEP_DIFF 25
#define ELEV_STEP_TOL  5
#define ELEV_BUMP_TAU  0.05
#define ELEV_BUMP_THR  0.5
#define ELEV_STEP_SPEED 1 //in ??

#define CLAMP_OPEN_POS   140
#define CLAMP_CLOSED_POS 120

#define CHERRY_UP_POS   180
#define CHERRY_DOWN_POS 90


/* PINS */
//TODO with elec

#define DOORS_LEFT_PIN 21
#define DOORS_RIGHT_PIN 20

#define MOTOR_INTERFACE_TYPE 1 //arduino connected to a stepper driver with 2 driver pins
#define ELEV_DIR_PIN 3
#define ELEV_STEP_PIN 4
#define ELEV_BUMP_UP_PIN 0
#define ELEV_BUMP_DOWN_PIN 0

#define CLAMP_PIN 6

#define CHERRY_PIN 5

#define ULTRASONIC_L_TRIG_PIN 23
#define ULTRASONIC_L_ECHO_PIN 22
#define ULTRASONIC_R_TRIG_PIN 7
#define ULTRASONIC_R_ECHO_PIN 8

#endif

/* OTHER */
#define ULTRASONIC_MEASURE_INTERVAL 50 //ms
#define ULTRASONIC_TRIG_INTERVAL 10 //us
#define ULTRASONIC_TIME_TO_DIST 58200 // us/cm