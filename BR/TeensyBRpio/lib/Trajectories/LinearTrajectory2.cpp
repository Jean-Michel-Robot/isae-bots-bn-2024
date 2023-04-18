/*
   
*/

#include <LinearTrajectory2.hpp>

#include <Arduino.h>


LinearTrajectory2::LinearTrajectory2(float initialGoalSpeed, float initialAccelParam)
{
    goalSpeed = initialGoalSpeed;
    accelParam = initialAccelParam;

    // goalSpeed = 1.0;  // m/s  //TODO à set par le HN
    // accelParam = 1.0;  // m/s^2  //TODO à set par le HN
    // currentSpeed = 0.0;

    // d_current = 0.0;

    // float accelParam = 0.2;

    //TODO optim
    x0 = 0.0;
    y0 = 0.0;
    xdest = 0.0;
    ydest = 0.0;
    theta0 = 0.0;
    Dtotale = 0.0;


}


void LinearTrajectory2::setDest (float x0, float y0, float xdest, float ydest) {
    this->x0 = x0;
    this->y0 = 0;
    this->xdest = xdest;
    this->ydest = ydest;

    Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    theta0 = atan2(ydest - y0, xdest - x0);  // returned
 
}


bool LinearTrajectory2::detectEndRamp() {
    return ( (Dtotale - d_current) < 0.5*currentSpeed*currentSpeed/accelParam );
}



void LinearTrajectory2::modifyVars(float *q) {

    q[0] = x0 + s*(xdest - x0);
    q[1] = y0 + s*(ydest - y0);
    q[2] = theta0;

    q[3] = currentSpeed;
    q[4] = 0.0;
}

// void LinearTrajectory2::setGoalSpeed(float goalSpeed) {
//     this->goalSpeed = goalSpeed;
//     rampSpeed.changeGoalSpeed(goalSpeed);
// }