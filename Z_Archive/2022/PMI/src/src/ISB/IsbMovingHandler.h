/*
    Allows for the creation of IsbMovingHandler objects that will handle the trigger of the moving tests
*/

#ifndef ISBMOVINGHANDLER_H
#define ISBMOVINGHANDLER_H

#include <Arduino.h>
#include "IsbHwHandler.h"
#include "IsbBasicHandler.h"
#include "../Timer/Timer.h"

#define MOVING_TIMEOUT 10 // s

#define ORDER_STOP 2
#define ORDER_RESET 3
#define ORDER_GOING 8 // backward
#define ORDER_COMING_BACK 7 //forward

#define MOVING_DISTANCE -100 // mm

class IsbMovingRos;

class IsbMovingHandler
{
public:
    enum MovingState
    {
        STATE_POS_NOT_SET = -1,
        STATE_IDLE = 0,
        STATE_BEGIN = 1,
        STATE_GOING = 2,
        STATE_ON_POINT = 3,
        STATE_COMING_BACK = 4,
        STATE_ERROR = 5,
        STATE_TIMEOUT = 6
    };

    IsbMovingHandler(IsbHwHandler *isbHwHandler,
                     uint8_t posMoving);

    IsbHwHandler *getIsbHwHandler();

    void setStartingPos(float startingPosX, float startingPosY, float startingPosTheta); // called by ROS on reset

    bool isPositionSet();

    void setIsbMovingRos(IsbMovingRos *isbMovingRos);

    void movingStateEnter(MovingState state);

    void nextMovingState(); // called by ROS on okTurn signal

    void handleSwitchStateChanges();

    void setup();

    void loop();
    
private:
    IsbHwHandler *m_isbHwHandler; // isb hardware handler

    uint8_t m_posMoving; // position of moving switch

    uint8_t m_maskMoving; // mask that gives the moving switch position

    bool m_movingSwitch; // Moving switch

    float m_startingPosX;
    float m_startingPosY;
    float m_startingPosTheta;

    float m_testingPosX;
    float m_testingPosY;
    float m_testingPosTheta;

    bool m_moving;
    MovingState m_currentMovingState; // moving state
                                      //updated by ROS on okTurn signal

    Timer m_timerMovingTimeout;

    // Ros communication
    IsbMovingRos *m_isbMovingRos;
};

#endif