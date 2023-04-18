/*
   
*/

#include <RotationTrajectory.hpp>

#include <Arduino.h>


RotationTrajectory::RotationTrajectory(float initialGoalSpeed, float initialAccelParam)
{
    goalSpeed = initialGoalSpeed;
    accelParam = initialAccelParam;

    // goalSpeed = 1.0;  // m/s  //TODO à set par le HN
    // accelParam = 1.0;  // m/s^2  //TODO à set par le HN
    // currentSpeed = 0.0;

    // d_current = 0.0;

    // float accelParam = 0.2;

    thetaDest = 0.0;

}

// A faire après avoir set la position du robot
void RotationTrajectory::setDest(float thetaDest) {
    this->thetaDest = thetaDest;

    float RADIUS = 0.10;  //TODO param

    Dtotale = RADIUS * abs(thetaDest - theta0);
    // theta0 = atan2(ydest - y0, xdest - x0);
 
}


bool RotationTrajectory::detectEndRamp() {
    return ( (Dtotale - d_current) < 0.5*currentSpeed*currentSpeed/accelParam );
}



void RotationTrajectory::updateTrajectoryState() {

    x = x0;
    y = y0;
    theta = theta0 + s*(thetaDest - theta0); //TODO pb du passage par -pi;pi

    V = 0.0;
    omega = currentSpeed;
    
}
