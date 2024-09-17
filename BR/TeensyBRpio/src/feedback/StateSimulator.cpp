#include "feedback/StateSimulator.hpp"

#include "defines.hpp"
#include "utils/clock.h"
#include "utils/math.h"

#define TICK_INTERVAL 1

StateSimulator &StateSimulator::instance()
{
    static StateSimulator instance;
    return instance;
}

void StateSimulator::setLeftMotorSpeed(float speed)
{
    m_leftMotorSpeed = speed;
}
void StateSimulator::setRightMotorSpeed(float speed)
{
    m_rightMotorSpeed = speed;
}
void StateSimulator::update()
{

    auto time = millis();
    if (m_lastTick == 0)
    {
        m_lastTick = time;
    }
    int ticks = (int)((time - m_lastTick) / TICK_INTERVAL);
    for (int i = 0; i < ticks; i++)
    {
        m_lastTick += TICK_INTERVAL;
        double rotSpeed = (m_rightMotorSpeed - m_leftMotorSpeed) / WHEEL_DISTANCE;
        double linSpeed = (m_leftMotorSpeed + m_rightMotorSpeed) / 2;
        double linOffset = linSpeed * TICK_INTERVAL / 1000.0;

        x += cos(m_position.theta) * linOffset;
        y += sin(m_position.theta) * linOffset;
        theta += rotSpeed * TICK_INTERVAL / 1000.0;
    }
    m_position = Position2D<Meter>((float)x, (float)y, (float)theta);
}

void StateSimulator::resetPosition(Position2D<Millimeter> pos)
{
    PositionFeedback::resetPosition(pos);
    x = m_position.x;
    y = m_position.y;
    theta = m_position.theta;
}

#ifdef __SIMU__

PositionFeedback &PositionFeedback::instance()
{
    return StateSimulator::instance();
}

#endif