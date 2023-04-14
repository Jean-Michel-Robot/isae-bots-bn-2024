/*
   
*/

#include <LinearTrajectory.hpp>

#define D_RAMPE 0.2

LinearTrajectory::LinearTrajectory()
{

    goalSpeed = 0.1;
    vd = 0.0;

    d_current = 0.0;

    float accelParam = 0.2;

    rampSpeed = Ramp(accelParam);  //TODO : static object or change it to dynamic ?
                                   // Or LinearTrajectory is dynamic but this is static ?
}


void LinearTrajectory::setDest(float x0, float y0, float xdest, float ydest) {
    this->x0 = x0;
    this->y0 = 0;
    this->xdest = xdest;
    this->ydest = ydest;

    Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    theta0 = atan2(ydest - y0, xdest - x0);  // returned
 
}


void LinearTrajectory::beginTrajectory(uint32_t t0) {
    this->t0 = t0;

    rampSpeed.beginRamp(t0, goalSpeed);

}



// Retourne (xd(t), yd(t), thetad(t)) dans une Position2D
Position2D LinearTrajectory::getPointAtTime(uint32_t current_time)  //TODO general class for this 
{

    // On récupère V(t) de la rampe


    d_current = goalSpeed * (current_time - t0);

    s = d_current / Dtotale;

    Position2D pos = Position2D(0.0, 0.0, 0.0);

    return pos;
}

float* LinearTrajectory::getVelAndTheta(uint32_t current_time) {
    d_current = goalSpeed*(current_time - t0);


    // float res[2] = {vd, theta0};
    return nullptr;
}