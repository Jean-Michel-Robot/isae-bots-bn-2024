/*
 * z_SetupLoop.cpp
 * Setup and loop functions
 */

#include "src/ISB/IsbBasicRos.h"
#include "src/ISB/IsbMovingRos.h"

#include "a_parameters.h"
#include "a_pins.h"

#include "ResistanceReaderROS.h"
#include "FlipperROS.h"
#include "RepliqueROS.h"
#include "RealStatuetteROS.h"
#include "UltrasonicRos.h"

ros::NodeHandle nh;

/* Isb */
IsbHwHandler isbHwHandler = IsbHwHandler(ISB_DATA_PIN, ISB_CLOCK_PIN, ISB_LATCH_PIN, ISB_LED_PIN);
IsbBasicHandler isbBasicHandler = IsbBasicHandler(&isbHwHandler,
                                                  ISB_POS_TIRETTE, ISB_POS_SIDE, ISB_POS_STRAT,
                                                  ISB_NB_STRAT);
IsbBasicRos isbBasicRos = IsbBasicRos(&nh, &isbBasicHandler);
IsbMovingHandler isbMovingHandler = IsbMovingHandler(&isbHwHandler,
                                                     ISB_POS_MOVING);
IsbMovingRos isbMovingRos = IsbMovingRos(&nh, &isbMovingHandler);

/* Resistance */
int servoReadLeftPositions[2] = {SERVO_READ_L_POSITION_RETRACTED,SERVO_READ_L_POSITION_DEPLOYED};
int servoReadRightPositions[2] = {SERVO_READ_R_POSITION_RETRACTED,SERVO_READ_R_POSITION_DEPLOYED};
ResistanceReader resistanceReader = ResistanceReader(RESISTANCE_READ_PIN,
                                                     SERVO_READ_L_PIN, servoReadLeftPositions,
                                                     SERVO_READ_R_PIN, servoReadRightPositions);

ResistanceReaderROS rrROS = ResistanceReaderROS(&resistanceReader, &nh);
                                                     

int servoFlipLeftPositions[2] = {SERVO_FLIP_L_POSITION_RETRACTED,SERVO_FLIP_L_POSITION_DEPLOYED};
int servoFlipRightPositions[2] = {SERVO_FLIP_R_POSITION_RETRACTED,SERVO_FLIP_R_POSITION_DEPLOYED};
Flipper flipper = Flipper(SERVO_FLIP_L_PIN, servoFlipLeftPositions,
                          SERVO_FLIP_R_PIN, servoFlipRightPositions); 

FlipperROS flipperROS = FlipperROS(&flipper, &nh);

Replique replique = Replique(SERVO_REPLIQUE_PUSH_PIN, SERVO_REPLIQUE_PUSH_POSITION_NORMAL, SERVO_REPLIQUE_PUSH_POSITION_PUSHED,
                             SERVO_REPLIQUE_BLOCK_PIN, SERVO_REPLIQUE_BLOCK_POSITION_UNLOCKED, SERVO_REPLIQUE_BLOCK_POSITION_LOCKED);

RepliqueROS repliqueROS = RepliqueROS(&nh, &replique);

RealStatuette realStatuette = RealStatuette(SERVO_REAL_STATUETTE_LEFT_PIN, SERVO_REAL_STAT_LEFT_POSITION_OPENED, SERVO_REAL_STAT_LEFT_POSITION_CLOSED,
                                            SERVO_REAL_STATUETTE_RIGHT_PIN, SERVO_REAL_STAT_RIGHT_POSITION_OPENED, SERVO_REAL_STAT_RIGHT_POSITION_CLOSED);

RealStatuetteROS realStatROS = RealStatuetteROS(&nh, &realStatuette);

UltrasonicSensor *ultrasonicSensorR = new UltrasonicSensor("R", SONAR_PIN_1, SONAR_BASE_1, SONAR_COEFF_1, SONAR_TIME_BEFORE_REFRESH);
UltrasonicSensor *ultrasonicSensorL = new UltrasonicSensor("L", SONAR_PIN_2, SONAR_BASE_2, SONAR_COEFF_2, SONAR_TIME_BEFORE_REFRESH);

UltrasonicSensor *ultrasonicSensorArray[] = {ultrasonicSensorR, ultrasonicSensorL};

UltrasonicRos ultrasonicRos = UltrasonicRos(&nh, SONAR_ROS_NB_ULTRASONIC_SENSORS, ultrasonicSensorArray, SONAR_ROS_TIME_PUB_DISTANCES);

void setup(){
    
    nh.initNode();

    /* Isb */
    isbHwHandler.setup();

    isbBasicHandler.setup();
    isbBasicRos.setup();

    isbMovingHandler.setup();
    isbMovingRos.setup();

    /* Resistance */
    rrROS.setup();
    flipperROS.setup(); 

    /*Replique and statuette*/
    repliqueROS.setup();
    realStatROS.setup();
 

    /* Ultrasonic sonars */
    for (int i = 0; i < SONAR_ROS_NB_ULTRASONIC_SENSORS; i++)
    {
        ultrasonicSensorArray[i]->setup();
    }

    ultrasonicRos.setup(); 

}

void loop(){

    nh.spinOnce();

    /* Isb */
    isbHwHandler.loop();

    isbBasicHandler.loop();
    isbBasicRos.loop();

    isbMovingHandler.loop();
    isbMovingRos.loop();

    /* Resistance */
    rrROS.loop();
    flipperROS.loop();

    /*Replique and statuette*/
    repliqueROS.loop();
    realStatROS.loop();

    /* Ultrasonic sonars */
    for (int i = 0; i < SONAR_ROS_NB_ULTRASONIC_SENSORS; i++)
    {
        ultrasonicSensorArray[i]->loop();
    }

    ultrasonicRos.loop();
 
}
