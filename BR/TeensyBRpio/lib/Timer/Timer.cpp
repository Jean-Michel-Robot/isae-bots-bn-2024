#include <Timer.hpp>

std::vector<Timer*> Timer::s_timerList;

Timer::Timer(float time)
{
    m_timerLength = floor(time * 1e3);
    s_addTimerToList(this);

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

void Timer::resetAllTimers()
{
    for(Timer* &timer : s_timerList)
    {
        timer->reset();
    }
}

void Timer::s_addTimerToList(Timer* timer)
{
    s_timerList.push_back(timer);
}





