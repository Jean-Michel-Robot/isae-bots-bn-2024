/*
   
*/

#include "Trajectories/RotationTrajectory.hpp"

#include <Arduino.h>
#include <GeometricTools.hpp>

#include "ROS.hpp"
#include "main_loop.hpp"


RotationTrajectory::RotationTrajectory(float initialGoalSpeed, float initialAccelParam)
{
    goalSpeed = initialGoalSpeed;
    accelParam = initialAccelParam;

    thetaDest = 0.0;

    trajectoryType = TrajectoryType::TRAJ_ROTATION;
}

// A faire après avoir set la position du robot
void RotationTrajectory::setDest(OrderType order) {
    this->thetaDest = order.theta;

    Dtotale = ASSERV_ALPHA * abs(thetaDest - theta0); //TODO : pb de modulo
    // theta0 = atan2(ydest - y0, xdest - x0);

}


bool RotationTrajectory::detectEndRamp() {
    return ( (Dtotale - d_current) < 0.5*currentSpeed*currentSpeed/accelParam );
}



void RotationTrajectory::updateTrajectoryState() {

    x = x0;
    y = y0;
    // theta = theta0 + s*(thetaDest - theta0); //TODO pb du passage par -pi;pi

    // en fait une différence d'angles se trouve toujours entre -pi et pi (angles opposés)
    // donc il faut passer la différence entre -pi et pi
    // Manuellement :
    float angle_diff = thetaDest - theta0;
    
    if (angle_diff > PI) {
        angle_diff = angle_diff - PI;
    }
    else if (angle_diff < -PI) {
        angle_diff = angle_diff + PI;
    }

    theta = theta0 + s*angle_diff; //TODO pb du passage par -pi;pi

    V = 0.0;
    omega = currentSpeed;
    
}