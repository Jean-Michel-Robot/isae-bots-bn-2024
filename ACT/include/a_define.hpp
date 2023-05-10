#ifndef DEFINES_HPP
#define DEFINES_HPP

/* POSITIONS */
//TODO

#define DOORS_LEFT_OPEN_POS    0
#define DOORS_LEFT_CLOSED_POS  90
#define DOORS_RIGHT_OPEN_POS   0
#define DOORS_RIGHT_CLOSED_POS 90

#define ELEV_STEP_ZERO 0
#define ELEV_STEP_DIFF 10

#define CLAMP_OPEN_POS   50
#define CLAMP_CLOSED_POS 60

#define CHERRY_UP_POS   40
#define CHERRY_DOWN_POS 60


/* PINS */
//TODO with elec

#define DOORS_LEFT_PIN 5
#define DOORS_RIGHT_PIN 6

#define MOTOR_INTERFACE_TYPE 1 //arduino connected to a stepper driver with 2 driver pins
#define ELEV_DIR_PIN 3
#define ELEV_STEP_PIN 4

#define CLAMP_PIN 20

#define CHERRY_PIN 21

#define ULTRASONIC_L_TRIG_PIN 22
#define ULTRASONIC_L_ECHO_PIN 23
#define ULTRASONIC_R_TRIG_PIN 7
#define ULTRASONIC_R_ECHO_PIN 8

#endif

/* OTHER */
#define ULTRASONIC_MEASURE_INTERVAL 50 //ms
#define ULTRASONIC_TRIG_INTERVAL 10 //us
#define ULTRASONIC_TIME_TO_DIST 58200 // us/cm