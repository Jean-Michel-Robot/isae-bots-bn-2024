/*
   
*/

#include "Trajectories/LinearTrajectory.hpp"

#include <Arduino.h>

#include "ROS.hpp"
#include "main_loop.hpp"


LinearTrajectory::LinearTrajectory(float initialGoalSpeed, float initialAccelParam)
{
    goalSpeed = initialGoalSpeed;
    accelParam = initialAccelParam;

    xdest = 0.0; ydest = 0.0;

    trajectoryType = TrajectoryType::TRAJ_LINEAR;
}

// A faire après avoir set la position du robot
void LinearTrajectory::setDest(OrderType order) {
    this->xdest = order.x;
    this->ydest = order.y;

    Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    theta0 = atan2(ydest - y0, xdest - x0);  // plus exact que le theta du robot

}


bool LinearTrajectory::detectEndRamp() {
    return ( (Dtotale - d_current) < 0.5*currentSpeed*currentSpeed/accelParam );
}



void LinearTrajectory::updateTrajectoryState() {

    x = x0 + s*(xdest - x0);
    y = y0 + s*(ydest - y0);
    theta = theta0;

    V = currentSpeed;
    omega = 0.0;
    
}
