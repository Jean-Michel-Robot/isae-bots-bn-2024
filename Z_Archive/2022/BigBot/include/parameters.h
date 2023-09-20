#ifndef PARAMETERS_H
#define PARAMETERS_H

/* Ultrasonic sonars */

#define SONAR_BASE_1 100.0 // mm
#define SONAR_COEFF_1 1.023

#define SONAR_BASE_2 100.0 // mm
#define SONAR_COEFF_2 1.023

#define SONAR_TIME_BEFORE_REFRESH 0.01 // s

#define SONAR_ROS_NB_ULTRASONIC_SENSORS 2
#define SONAR_ROS_TIME_PUB_DISTANCES 0.1 // s

#define SONAR_PIN_1 27
#define SONAR_PIN_2 26

/* ISB pos */
#define ISB_POS_TIRETTE 0
#define ISB_POS_SIDE 1
#define ISB_POS_MOVING 5

/* now indacates the first switch used for strategy determination (starting from 0) :
 * e.g. with 6, the last two switches are used for the stategy so that strategy goes from 0 to 3
 */
#define ISB_POS_STRAT 6 

#define ISB_NB_STRAT 3 //useless now (kept out of laziness)

#define ISB_DATA_PIN 36
#define ISB_CLOCK_PIN 16
#define ISB_LATCH_PIN 15
#define ISB_LED_PIN 14

#endif