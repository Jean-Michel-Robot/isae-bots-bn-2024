/*
   
*/

#include "Trajectories/BezierTrajectory.hpp"

#include <Arduino.h>

#include "ROS.hpp"
#include "main_loop.hpp"


BezierTrajectory::BezierTrajectory(float initialGoalSpeed, float initialAccelParam)
{
    goalSpeed = initialGoalSpeed;
    accelParam = initialAccelParam;

    xdest = 0.0; ydest = 0.0;

    trajectoryType = TrajectoryType::TRAJ_LINEAR;
}

// A faire après avoir set la position du robot
void BezierTrajectory::setDest(Position2D orderInfo) {
    this->xdest = orderInfo.x;
    this->ydest = orderInfo.y;

    Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    // theta0 = atan2(ydest - y0, xdest - x0);  // plus exact que le theta du robot

}



void BezierTrajectory::updateTrajectoryState() {

    x = x0 + s*(xdest - x0);
    y = y0 + s*(ydest - y0);
    theta = theta0;

    V = currentSpeed;
    omega = 0.0;

    // Update des vitesses absolues du goal point en linéaire
    ppoint_d[0] = V*cos(theta);
    ppoint_d[1] = V*sin(theta);
}


