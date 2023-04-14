/*
   
*/

#include <LinearTrajectory.hpp>

#define D_RAMPE 0.2

LinearTrajectory::LinearTrajectory(uint32_t t_0, float x0, float y0, float xdest, float ydest)
{
    this->t_0 = t_0;

    V = 0.1;
    vd = 0.0;

    d_current = 0.0;

    Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    
    theta0 = atan2(ydest - y0, xdest - x0);  // returned

    velTheta[0] = 0.0;
    velTheta[1] = 0.0;

    // rampSpeed = Ramp(2.0);
}

// Retourne (xd(t), yd(t), thetad(t)) dans une Position2D
Position2D LinearTrajectory::getPointAtTime(uint32_t current_time)  //TODO general class for this 
{

    // On récupère V(t) de la rampe


    d_current = V * (current_time - t_0);

    s = d_current / Dtotale;

    Position2D pos = Position2D(0.0, 0.0, 0.0);

    return pos;
}

float* LinearTrajectory::getVelAndTheta(uint32_t current_time) {
    d_current = V*(current_time - t_0);


    // float res[2] = {vd, theta0};
    return nullptr;
}