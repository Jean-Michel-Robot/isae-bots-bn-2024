/*
   
*/

#include <LinearTrajectory.hpp>

#include <Arduino.h>


LinearTrajectory::LinearTrajectory(float initialGoalSpeed, float initialAccelParam)
{
    goalSpeed = initialGoalSpeed;
    accelParam = initialAccelParam;

    // goalSpeed = 1.0;  // m/s  //TODO à set par le HN
    // accelParam = 1.0;  // m/s^2  //TODO à set par le HN
    // currentSpeed = 0.0;

    // d_current = 0.0;

    // float accelParam = 0.2;

    xdest = 0.0; ydest = 0.0;

}

// A faire après avoir set la position du robot
void LinearTrajectory::setDest(float xdest, float ydest) {
    this->xdest = xdest;
    this->ydest = ydest;

    Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    theta0 = atan2(ydest - y0, xdest - x0);  // returned
 
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
