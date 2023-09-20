#include "IsbMovingRos.h"

IsbMovingHandler *IsbMovingRos::m_isbMovingHandler = NULL;

IsbMovingRos::IsbMovingRos(ros::NodeHandle *nh, IsbMovingHandler *isbMovingHandler)
    : subIsbOkPosition("okPosition", isbOkPositionCb),
    subIsbNextPosition("nextPositionTeensy", isbNextPositionCb),
    pubIsbMovingOrder("nextPositionTeensy", &isbMovingOrder)
{
    this->m_nh = nh;
    this->m_isbMovingHandler = isbMovingHandler;
}

void IsbMovingRos::isbNextPositionCb(const geometry_msgs::Quaternion &nextPosMsg)
{
    if (m_isbMovingHandler->getIsbHwHandler()->isMatchStarted())
        return;

    if (nextPosMsg.w == ORDER_RESET)
    {
        m_isbMovingHandler->setStartingPos(nextPosMsg.x,nextPosMsg.y, nextPosMsg.z);
    }
}

void IsbMovingRos::isbOkPositionCb(const std_msgs::Int16 &okPosMsg)
{
    if (!m_isbMovingHandler->isPositionSet() || m_isbMovingHandler->getIsbHwHandler()->isMatchStarted())
        return;

    if (okPosMsg.data == 2) // okTurn
    {
        m_isbMovingHandler->nextMovingState();
    }
    else if (okPosMsg.data == 0) // control error
    {
        m_isbMovingHandler->movingStateEnter(IsbMovingHandler::STATE_ERROR);
    }
}

void IsbMovingRos::publishIsbMovingOrder(float x, float y, float theta, int orderId)
{
//     if (!m_isbMovingHandler->isPositionSet() || m_isbMovingHandler->getIsbHwHandler()->isMatchStarted())
//         return;

//     isbMovingOrder.x = x;
//     isbMovingOrder.y = y;
//     isbMovingOrder.z = theta;
//     isbMovingOrder.w = orderId;

//     pubIsbMovingOrder.publish(&isbMovingOrder);
}

void IsbMovingRos::setup()
{
    m_isbMovingHandler->setIsbMovingRos(this);

    m_nh->subscribe(subIsbNextPosition);
    m_nh->subscribe(subIsbOkPosition);
    m_nh->advertise(pubIsbMovingOrder);
}

void IsbMovingRos::loop()
{
}