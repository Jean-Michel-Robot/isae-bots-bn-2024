
#include "main_loop.hpp"

#include <Arduino.h>

#include <Motors.hpp>
#include <LED.hpp>
#include <Logger.h>


#include "ROS.hpp"
#include "OdosPosition.hpp"
#include "Trajectories/LinearTrajectory.hpp"
#include "Trajectories/RotationTrajectory.hpp"
#include "Asserv.hpp"
#include "BrSM/BrSMWrapper.hpp"



ROS* p_ros = NULL;
OdosPosition* p_odos = NULL;
BlinkLED* p_blink = NULL;
// LED* led_instance = NULL;

LinearTrajectory* p_linearTrajectory = NULL;
RotationTrajectory* p_rotationTrajectory = NULL;

Asserv* p_asserv = NULL;
BrSMWrapper* p_sm = NULL;

Logger* p_logger = NULL;

void setup() {

    // Serial.begin(9600);
    // delay(500);
    // Serial.println("Setup");


    motors_init();

    // Instanciation des classes en dynamique
    p_ros = new ROS();
    p_odos = new OdosPosition();
    p_blink = new BlinkLED();

    // led_instance = new LED();

    p_linearTrajectory = new LinearTrajectory(0.6 * MAX_LINEAR_GOAL_SPEED, DEFAULT_LINEAR_ACCEL_PARAM);
    p_rotationTrajectory = new RotationTrajectory(0.6 * MAX_ROTATION_GOAL_SPEED, DEFAULT_ROTATION_ACCEL_PARAM);

    // p_linearTrajectory->setRobotPos(100, 100, 0);
    // p_linearTrajectory->setDest(0.0, 0.0);
    // p_linearTrajectory->beginTrajectory( micros() );

    p_asserv = new Asserv(5.0, 10.0, 1.0);  //TODO réglage des gains

    p_sm = new BrSMWrapper();

}

bool led_on = false;
unsigned long timer_old = 0;
// Un tour -> 8192 ticks à vue d'oeil
// int32_t odoL;
// int32_t odoR;

uint32_t loop_timer = 0.0;

float current_speed;

void loop() {

    // uint32_t t = micros();


    p_odos->loop();

    p_blink->loop();


    p_sm->loop();


    // Log update
    if (millis() - loop_timer > LOG_PERIOD) {

        // if (p_sm->getCurrentState() != BR_IDLE) {
        //     p_ros->logPrint(INFO, "Speed : "+String(p_sm->getCurrentTargetSpeed()));
        //     // //Serial.println(p_odos->getRobotPosition().toString());
        //     // //Serial.println("Current BR state : " + p_sm->getCurrentStateStr());
        //     // //Serial.println("Current ramp state : " + p_sm->currentTrajectory->rampSpeed.rampSM.getCurrentStateStr());

        //     //Serial.println(p_sm->currentTrajectory->getGoalPoint().toString());
        //     //p_ros->sendDebug();

        //     p_ros->logPrint(INFO, "s : "+String(p_sm->currentTrajectory->s));

        //     p_ros->logPrint(INFO, "Error : [" + String(p_asserv->error[0]) +
        //                               ", " + String(p_asserv->error[1]) + "]");

        //     p_ros->logPrint(INFO, "Cmds : [" + String(p_asserv->cmd_coordspoint[0]) +
        //                               ", " + String(p_asserv->cmd_coordspoint[1]) + "]");

        //     p_ros->logPrint(INFO, "Cmd R : " + String(p_asserv->m_rightWheelSpeed) +
        //                               " | Cmd L : " + String(p_asserv->m_leftWheelSpeed));
        // }

        Logger::setFieldValue(millis(), Logger::currentTime);

        Position2D robotPos = p_odos->getRobotPosition();
        Logger::setFieldValue(robotPos.x, Logger::robotPosX);
        Logger::setFieldValue(robotPos.y, Logger::robotPosY);
        Logger::setFieldValue(robotPos.theta, Logger::robotPosTheta);

        Position2D goalPos = p_sm->getCurrentGoalPos();
        Logger::setFieldValue(goalPos.x, Logger::goalPointPosX);
        Logger::setFieldValue(goalPos.y, Logger::goalPointPosY);
        Logger::setFieldValue(goalPos.theta, Logger::goalPointPosTheta);

        float *goalSpeed = p_sm->currentTrajectory->getTrajectoryAbsoluteSpeed();
        Logger::setFieldValue(goalSpeed[0], Logger::goalPointSpeedX);
        Logger::setFieldValue(goalSpeed[1], Logger::goalPointSpeedY);

        Logger::setFieldValue(p_sm->currentTrajectory->s, Logger::trajectoryS);

        if (p_sm->currentTrajectory == NULL) {
            Logger::setFieldValue(0.0, Logger::goalSpeedLinear);
            Logger::setFieldValue(0.0, Logger::goalSpeedAngular);            
        }
        else if (p_sm->currentTrajectory->trajectoryType == TrajectoryType::TRAJ_LINEAR) {
            Logger::setFieldValue(p_sm->currentTrajectory->goalSpeed, Logger::goalSpeedLinear);
            Logger::setFieldValue(0.0, Logger::goalSpeedAngular);
       }
        else if (p_sm->currentTrajectory->trajectoryType == TrajectoryType::TRAJ_ROTATION) {
            Logger::setFieldValue(0.0, Logger::goalSpeedLinear);
            Logger::setFieldValue(p_sm->currentTrajectory->goalSpeed, Logger::goalSpeedAngular);
        }

        Logger::setFieldValue(p_asserv->error[0], Logger::asservErrorX);
        Logger::setFieldValue(p_asserv->error[1], Logger::asservErrorY);

        Logger::setFieldValue(p_asserv->cmd_v, Logger::commandV);
        Logger::setFieldValue(p_asserv->cmd_omega, Logger::commandOmega);

        Logger::setFieldValue(p_asserv->m_rightWheelSpeed, Logger::commandeMotorR);
        Logger::setFieldValue(p_asserv->m_leftWheelSpeed, Logger::commandeMotorL);

        if (p_sm->currentTrajectory == NULL) {
            Logger::setFieldValue(0.0, Logger::rampSpeed);
            Logger::setFieldValue(0.0, Logger::rampState);
        }
        else {
            Logger::setFieldValue(p_sm->currentTrajectory->rampSpeed.rampSM.getCurrentSpeed(), Logger::rampSpeed);
            Logger::setFieldValue(p_sm->currentTrajectory->rampSpeed.rampSM.getCurrentState(), Logger::rampState);
        }

        Logger::setFieldValue(p_sm->getCurrentState(), Logger::BrState);

        loop_timer = millis();
    }

    p_ros->loop();


    return;

    // Commands for debugging
    if (Serial.available()) {
        char c = Serial.read();

        if (c == 't') {
            Serial.println("Test input");
        }

        else if (c == 'r') {
            p_ros->logPrint(INFO, "Received get ready event");

            BrGetReadyEvent brGetReadyEvent;
            p_sm->send_event(brGetReadyEvent);    
        }

        else if (c == 'o') {
            //Serial.println("Received order request");

            OrderEvent orderEvent;
            orderEvent.order.x = 0.5;
            orderEvent.order.y = 0.0;
            orderEvent.order.theta = 0.0;
            orderEvent.order.goalType = GoalType::TRANS;  // on essaie direct le depl linéaire

            p_sm->send_event(orderEvent);

        }


        else if (c == 's') {
            //Serial.println("Send goal speed change event of 0.5");

            GoalSpeedChangeEvent goalSpeedChangeEvent;
            goalSpeedChangeEvent.newSpeed = 0.5;

            p_sm->brSM.currentTrajectory->rampSpeed.rampSM.send_event(goalSpeedChangeEvent);
        }
        else if (c == 'd') {
            //Serial.println("Send goal speed change event of 0.1");

            GoalSpeedChangeEvent goalSpeedChangeEvent;
            goalSpeedChangeEvent.newSpeed = 0.1;

            p_sm->brSM.currentTrajectory->rampSpeed.rampSM.send_event(goalSpeedChangeEvent);
        }

        else if (c == 'b') {
            //Serial.println("Send emergency brake event");

            EmergencyBrakeEvent emergencyBrakeEvent;

            p_sm->brSM.currentTrajectory->rampSpeed.rampSM.send_event(emergencyBrakeEvent);
        }

        else if (c == 'e') {
            //Serial.println("Send end ramp event");

            EndRampEvent endRampEvent;

            p_sm->brSM.currentTrajectory->rampSpeed.rampSM.send_event(endRampEvent);
        }
    }

}
