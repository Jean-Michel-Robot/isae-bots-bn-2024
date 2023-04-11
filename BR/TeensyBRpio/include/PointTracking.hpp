#ifndef __H_POINT_TRACKING
#define __H_POINT_TRACKING

// #include <Arduino.h>
// #include "a_define.h"
#include <Position2D.h>


class PointTracking
{
public :
    PointTracking(); // constructeur sans type
    // TODO destructeur

    // updateLinearTraj()

    void calculatePointPos(uint32_t current_time);
    Position2D getPointPos();

// protected :


private:

    uint32_t t_0;  // begin time of a trajectory

    // linearTrajectory  // TODO be a child of a larger Trajectory class

    Position2D trackingPos;  // x_d, y_d, theta_d


};

#endif
