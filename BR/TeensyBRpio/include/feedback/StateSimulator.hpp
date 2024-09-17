#ifndef _STATE_SIMULATOR
#define _STATE_SIMULATOR

#include "feedback/PositionFeedback.hpp"
#include "geometry/Position2D.h"

class StateSimulator: public PositionFeedback
{
public:
    void setLeftMotorSpeed(float speed);
    void setRightMotorSpeed(float speed);

    static StateSimulator &instance();

protected:
    void update() override;

private:
    StateSimulator() = default;

    float m_leftMotorSpeed = 0;
    float m_rightMotorSpeed = 0;
    unsigned long m_lastTick = 0;

    double x = 0;
    double y = 0;
    double theta = 0;
};

#endif