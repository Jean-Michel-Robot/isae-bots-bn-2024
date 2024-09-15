#ifndef _POSITION_FEEDBACK_H
#define _POSITION_FEEDBACK_H

#include "geometry/Position2D.h"

class PositionFeedback
{
public:
    void loop();
    virtual void resetPosition(Position2D<Millimeter> pos);
    Position2D<Meter> getRobotPosition() const;

    static PositionFeedback &instance();

protected:
    virtual void update() = 0;
    virtual void sendPosition() const;
    Position2D<Meter> m_position;
    PositionFeedback() = default;

private:
    unsigned long m_timer_last_send = 0;
};

#endif