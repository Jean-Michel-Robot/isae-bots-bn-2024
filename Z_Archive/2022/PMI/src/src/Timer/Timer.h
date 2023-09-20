#ifndef __TIMER_H_INCLUDED
#define __TIMER_H_INCLUDED
#ifdef __linux__

//#include <string>
// include a rajouter pour compiler en C++ sur un PC
//#include <cmath>
#define min(A,B) ((A)>(B)? (B): (A))
#define sq(A) ((A)*(A))
#define abs(A) ((A)>0.0? (A) : -(A))
#endif

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

};

#endif
