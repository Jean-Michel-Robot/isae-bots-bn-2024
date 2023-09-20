#include "IsbBasicRos.h"

IsbBasicHandler *IsbBasicRos::m_isbBasicHandler = NULL;

IsbBasicRos::IsbBasicRos(ros::NodeHandle *nh, IsbBasicHandler *isbBasicHandler)
    : subIsbSetSinglePixel("isbSetSinglePixel", isbSetSinglePixelCb),
    subIsbReset("isbReset", isbResetCb),
    pubIsbMatchState("isbMatchState", &isbMatchState),
    pubIsbSide("isbSide", &isbSide),
    pubIsbStrat("isbStrat", &isbStrat)
{
    this->m_nh = nh;
    this->m_isbBasicHandler = isbBasicHandler;
}

void IsbBasicRos::isbSetSinglePixelCb(const geometry_msgs::Quaternion &pixelMsg)
{
    m_isbBasicHandler->getIsbHwHandler()->setSinglePixel((uint8_t)pixelMsg.x, (uint8_t)pixelMsg.y, (uint8_t)pixelMsg.z, (uint8_t)pixelMsg.w);
}

void IsbBasicRos::isbResetCb(const std_msgs::Empty &resetMsg)
{
    m_isbBasicHandler->getIsbHwHandler()->reset();
    m_isbBasicHandler->reset();
}

void IsbBasicRos::publishIsbMatchStarted()
{
    isbMatchState.data = 1;
    pubIsbMatchState.publish(&isbMatchState);
}

void IsbBasicRos::publishIsbSide(uint16_t side)
{
    isbSide.data = side;
    pubIsbSide.publish(&isbSide);
}

void IsbBasicRos::publishIsbStrat(uint16_t strat)
{
    isbStrat.data = strat;
    pubIsbStrat.publish(&isbStrat);
}

void IsbBasicRos::setup()
{
    m_isbBasicHandler->setIsbBasicRos(this);

    m_nh->subscribe(subIsbSetSinglePixel);
    m_nh->subscribe(subIsbReset);
    m_nh->advertise(pubIsbMatchState);
    m_nh->advertise(pubIsbSide);
    m_nh->advertise(pubIsbStrat);
}

void IsbBasicRos::loop()
{
}