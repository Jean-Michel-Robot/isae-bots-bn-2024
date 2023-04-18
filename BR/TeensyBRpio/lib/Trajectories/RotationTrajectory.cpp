/*
   
*/

#include <RotationTrajectory.hpp>

#include "defines.hpp"

RotationTrajectory::RotationTrajectory()
{

    goalSpeed = 1.0;
    vd = 0.0;

    d_current = 0.0;

    float accelParam = 0.2;

    rampSpeed = Ramp(accelParam);  //TODO : static object or change it to dynamic ?
                                   // Or LinearTrajectory is dynamic but this is static ?
}


void RotationTrajectory::setDest(float x0, float y0, float theta0, float thetaDest) {
    this->x0;
    this->y0;
    this->theta0 = theta0;
    this->thetaDest = thetaDest;

    Dtotale = RADIUS * abs(thetaDest - theta0);  // longueur d'un arc de cercle
    //TODO : les edge cases où les angles sont de part et d'autre de PI
}


void RotationTrajectory::beginTrajectory(uint32_t t0) {
    this->t0 = t0;

    rampSpeed.beginRamp(t0, goalSpeed);

}



// Retourne (xd(t), yd(t), thetad(t)) dans une Position2D
Position2D RotationTrajectory::getPointAtTime(uint32_t current_time)  //TODO general class for this 
{

    // On récupère V(t) de la rampe
    float current_speed = rampSpeed.updateRamp(current_time);

    d_current = goalSpeed * (current_time - t0);

    s = d_current / Dtotale;

    Position2D pos = Position2D(0.0, 0.0, 0.0);

    return pos;
}

float RotationTrajectory::getVelAndTheta(uint32_t current_time) {
    d_current = goalSpeed*(current_time - t0);

    float current_speed = rampSpeed.updateRamp(current_time);

    // float res[2] = {vd, theta0};
    return current_speed;
}