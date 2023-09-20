/**
 * @file a_parameters.h
 * @brief File that contains all the parameters for the robot
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

/* ISB pos */
#define ISB_POS_TIRETTE 0
#define ISB_POS_SIDE 1
#define ISB_POS_MOVING 5

/* now indacates the first switch used for strategy determination (starting from 0) :
 * e.g. with 6, the last two switches are used for the stategy so that strategy goes from 0 to 3
 */
#define ISB_POS_STRAT 6 

#define ISB_NB_STRAT 3 //useless now (kept out of laziness)


/* Resistance Reading */

#define RESISTANCE_READ_R0 900 //Convert numirical input into raw resistance value in ohm such as R = R0 * (1024/AnalogRead - 1) 

#define SERVO_READ_L_POSITION_DEPLOYED 35
#define SERVO_READ_L_POSITION_RETRACTED 137
#define SERVO_READ_R_POSITION_DEPLOYED 150
#define SERVO_READ_R_POSITION_RETRACTED 30
/* Resistance value intervals
 * Theorical values :
 * Purple -> 470
 * Yellow -> 1K
 * Cross  -> 4.7K
 */
#define RESISTANCE_YELLOW_MIN 800
#define RESISTANCE_YELLOW_MAX 1200
#define RESISTANCE_PURPLE_MIN 300 
#define RESISTANCE_PURPLE_MAX 590
#define RESISTANCE_CROSS_MIN  4000
#define RESISTANCE_CROSS_MAX  6000

#define RESISTANCE_TIMEOUT 1000      //timeout for the reader deployment
#define RESISTANCE_MEASURE_DELAY  25 //delay between measures
#define RESISTANCE_MEASURE_NUMBER 10 //total number of measures
#define RESISTANCE_MEASURE_NB_MIN 7 //minimal number of correct measures to state that the measuring is satisfaying


/* Resistance Flipping */

#define SERVO_FLIP_L_POSITION_DEPLOYED 40
#define SERVO_FLIP_L_POSITION_RETRACTED 156
#define SERVO_FLIP_R_POSITION_DEPLOYED 110
#define SERVO_FLIP_R_POSITION_RETRACTED 0

#define FLIPPER_TIMEOUT 500 //timeout for the flipper

/* Statuette and replique */

#define SERVO_REPLIQUE_BLOCK_POSITION_LOCKED 180
#define SERVO_REPLIQUE_BLOCK_POSITION_UNLOCKED 100
#define SERVO_REPLIQUE_PUSH_POSITION_NORMAL 136
#define SERVO_REPLIQUE_PUSH_POSITION_PUSHED 90
#define SERVO_REAL_STAT_LEFT_POSITION_OPENED 170
#define SERVO_REAL_STAT_LEFT_POSITION_CLOSED 65
#define SERVO_REAL_STAT_RIGHT_POSITION_OPENED 10
#define SERVO_REAL_STAT_RIGHT_POSITION_CLOSED 130

#define REPLIQUE_UNLOCK_TIMER_MS 200
#define REPLIQUE_DELIVER_DIVISION_MS 50 //time between each division of the movement for the delivering servo
#define REPLIQUE_DELIVER_NUMBER_ITERATIONS 20 //number of division
#define REPLIQUE_DELIVER_FINAL_TIMER_MS 300 //time to wait after the end of the movement of the servo


#define REAL_STAT_OPENED_TIMER_MS 750
#define REAL_STAT_CLOSED_TIMER_MS 500


/* Ultrasonic sonars */

#define SONAR_BASE_1 100.0 // mm
#define SONAR_COEFF_1 1.023

#define SONAR_BASE_2 100.0 // mm
#define SONAR_COEFF_2 1.023

#define SONAR_TIME_BEFORE_REFRESH 0.01 // s

#define SONAR_ROS_NB_ULTRASONIC_SENSORS 2
#define SONAR_ROS_TIME_PUB_DISTANCES 0.1 // s

#endif
