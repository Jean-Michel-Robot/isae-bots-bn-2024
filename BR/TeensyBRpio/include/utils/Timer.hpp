#ifndef __TIMER_H_INCLUDED
#define __TIMER_H_INCLUDED

#include <vector>

class Timer
{
public :
    Timer(float time);
    void setLength(float time);
    void reset();
    void start(unsigned long time); // in ms by default
    bool isExpired(unsigned long time) const;
    bool isStarted() const;
    bool startIfNotStartedAndTestExpiration(unsigned long time); // in ms by default

    static void resetAllTimers();
private :
    unsigned long m_timerLength = 0.0;
    unsigned long m_startTime = 0;
    bool m_isStarted = false;

    static std::vector<Timer*> s_timerList;
    static void s_addTimerToList(Timer* timer);

};

#endif
