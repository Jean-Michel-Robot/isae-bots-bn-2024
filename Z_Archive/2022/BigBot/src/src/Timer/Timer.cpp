#include "Timer.h"
#ifndef __linux__
#include <Arduino.h>
#endif

Timer::Timer(float time)
{
    m_timerLength = floor(time * 1e3);

}

void Timer::setLength(float time)
{
    m_timerLength = floor(time * 1e3);
}

void Timer::reset()
{
    m_isStarted = false;
}

void Timer::start(unsigned long time)
{
    m_isStarted = true;
    m_startTime = time;
}

bool Timer::isExpired(unsigned long time) const
{
    return !m_isStarted || time > m_startTime + m_timerLength;
}

bool Timer::isStarted() const
{
    return m_isStarted;
}

bool Timer::startIfNotStartedAndTestExpiration(unsigned long time)
{
    if(!isStarted())
    {
        start(time);
    }
    return isExpired(time);
}




