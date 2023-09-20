/*
    Allows for the creation of IsbMovingHandler objects that will handle the trigger of the moving tests
*/

#include "IsbMovingHandler.h"
#include "IsbMovingRos.h"

//constructor of IsbMovingHandler class
IsbMovingHandler::IsbMovingHandler(IsbHwHandler *isbHwHandler,
                                   uint8_t posMoving)
    : m_timerMovingTimeout(MOVING_TIMEOUT)
{
    this->m_isbHwHandler = isbHwHandler;

    this->m_posMoving = posMoving;

    this->m_maskMoving = (1 << posMoving);

    this->m_movingSwitch = 0;

    this->m_startingPosX = 0;
    this->m_startingPosY = 0;
    this->m_startingPosTheta = 0;

    this->m_testingPosX = 0;
    this->m_testingPosY = 0;
    this->m_testingPosTheta = 0;

    this->m_moving = false;
    this->m_currentMovingState = STATE_POS_NOT_SET;

    this->m_isbMovingRos = NULL;
}

IsbHwHandler *IsbMovingHandler::getIsbHwHandler()
{
    return m_isbHwHandler;
}

void IsbMovingHandler::setStartingPos(float startingPosX, float startingPosY, float startingPosTheta)
{
    if (m_isbHwHandler->isMatchStarted())
        return;

    m_startingPosX = startingPosX;
    m_startingPosY = startingPosY;
    m_startingPosTheta = startingPosTheta;

    // compute new testingPos
    m_testingPosX = m_startingPosX + MOVING_DISTANCE * cos(m_startingPosTheta);
    m_testingPosY = m_startingPosY + MOVING_DISTANCE * sin(m_startingPosTheta);
    m_testingPosTheta = m_startingPosTheta;

    if (m_currentMovingState == STATE_POS_NOT_SET)
    {
        movingStateEnter(STATE_IDLE);
    }
    else if (m_moving)
    {
        movingStateEnter(STATE_ERROR);
    }
}

bool IsbMovingHandler::isPositionSet()
{
    return m_currentMovingState != STATE_POS_NOT_SET;
}

void IsbMovingHandler::setIsbMovingRos(IsbMovingRos *isbMovingRos)
{
    m_isbMovingRos = isbMovingRos;
}

void IsbMovingHandler::nextMovingState() // called by ROS on okTurn
{
    if (m_currentMovingState == STATE_POS_NOT_SET || m_isbHwHandler->isMatchStarted())
        return;

    switch (m_currentMovingState)
    {
    case STATE_POS_NOT_SET:
        break;
    case STATE_IDLE:
        break;
    case STATE_BEGIN:
        movingStateEnter(STATE_GOING);
        break;
    case STATE_GOING:
        movingStateEnter(STATE_ON_POINT);
        break;
    case STATE_ON_POINT:
        movingStateEnter(STATE_COMING_BACK);
        break;
    case STATE_COMING_BACK:
        movingStateEnter(STATE_IDLE);
        break;
    case STATE_ERROR:
        break;
    case STATE_TIMEOUT:
        break;
    }
}

void IsbMovingHandler::movingStateEnter(MovingState state)
{
    if (m_isbHwHandler->isMatchStarted())
        return;

    m_currentMovingState = state;

    switch (m_currentMovingState)
    {
    case STATE_POS_NOT_SET:
        m_moving = false;
        m_timerMovingTimeout.reset();
        break;

    case STATE_IDLE:
        m_moving = false;
        m_timerMovingTimeout.reset();
        m_isbHwHandler->setSinglePixel(m_posMoving, 0, 255, 0); // update led color
        break;

    case STATE_BEGIN:
        m_moving = true;
        m_timerMovingTimeout.start(millis());
        // publish msg with first position
        if (m_isbMovingRos != NULL)
            m_isbMovingRos->publishIsbMovingOrder(m_testingPosX, m_testingPosY, m_testingPosTheta, ORDER_GOING);
        m_isbHwHandler->setSinglePixel(m_posMoving, 255, 255, 255);
        break;

    case STATE_GOING:
        m_isbHwHandler->setSinglePixel(m_posMoving, 0, 0, 255); // update led color
        break;

    case STATE_ON_POINT:
        // publish msg with first position
        if (m_isbMovingRos != NULL)
            m_isbMovingRos->publishIsbMovingOrder(m_startingPosX, m_startingPosY, m_startingPosTheta, ORDER_COMING_BACK);
        m_isbHwHandler->setSinglePixel(m_posMoving, 0, 255, 255); // update led color
        break;

    case STATE_COMING_BACK:
        m_isbHwHandler->setSinglePixel(m_posMoving, 255, 0, 255); // update led color
        break;

    case STATE_ERROR:
        m_moving = false;
        m_timerMovingTimeout.reset();
        if (m_isbMovingRos != NULL)
            m_isbMovingRos->publishIsbMovingOrder(0, 0, 0, ORDER_STOP); // sending stop
        m_isbHwHandler->setSinglePixel(m_posMoving, 255, 0, 0);         // update led color
        break;

    case STATE_TIMEOUT:
        m_moving = false;
        m_timerMovingTimeout.reset();
        if (m_isbMovingRos != NULL)
            m_isbMovingRos->publishIsbMovingOrder(0, 0, 0, ORDER_STOP); // sending stop
        m_isbHwHandler->setSinglePixel(m_posMoving, 255, 255, 0);       // update led color
        break;
    }
}

void IsbMovingHandler::handleSwitchStateChanges()
{
    //store old variables
    bool oldMovingSwitch = m_movingSwitch;

    //update variables with new value
    m_movingSwitch = (m_isbHwHandler->getSwitchesState() & m_maskMoving) >> m_posMoving;

    // don't do anything if pos are not set or if we are moving
    if (m_currentMovingState == STATE_POS_NOT_SET || m_moving)
        return;

    // going from off to on
    if (!oldMovingSwitch && m_movingSwitch)
    {
        if (m_currentMovingState == STATE_IDLE)
        {
            movingStateEnter(STATE_BEGIN);
        }
    }

    // going off in an error state
    else if (oldMovingSwitch && !m_movingSwitch)
    {
        movingStateEnter(STATE_IDLE);
    }
}

void IsbMovingHandler::setup()
{
    //update the moving switch state
    m_movingSwitch = (m_isbHwHandler->getSwitchesState() & m_maskMoving) >> m_posMoving;

    movingStateEnter(STATE_POS_NOT_SET);
}

void IsbMovingHandler::loop()
{
    if (m_isbHwHandler->isMatchStarted())
        return;

    handleSwitchStateChanges();

    if (m_timerMovingTimeout.isStarted() && m_timerMovingTimeout.isExpired(millis()))
    {
        // publish stop
        movingStateEnter(STATE_TIMEOUT);
    }
}