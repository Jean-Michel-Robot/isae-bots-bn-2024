#include "feedback/StateSimulator.hpp"
#include "defines.hpp"
#include "motors.hpp"

#define TICK_INTERVAL 10

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
    int ticks = (int)((float)(time - m_lastTick) / TICK_INTERVAL);
    for (int i = 0; i < ticks; i++)
    {
        m_lastTick += TICK_INTERVAL;

        double rotSpeed = (m_rightMotorSpeed - m_leftMotorSpeed) / WHEEL_DISTANCE;
        double linSpeed = (m_leftMotorSpeed + m_rightMotorSpeed) / 2;

        double linOffset = linSpeed * TICK_INTERVAL / 1000.0;
        m_position.x += (float)cos(m_position.theta * linOffset);
        m_position.y += (float)sin(m_position.theta * linOffset);

        m_position.theta += rotSpeed * TICK_INTERVAL / 1000.0;
    }
}

#ifdef __SIMU__

PositionFeedback &PositionFeedback::instance()
{
    StateSimulator::instance();
}

#endif