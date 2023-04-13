/*
   
*/

#include "LinearTrajectory.hpp"

#define D_RAMPE 0.2

LinearTrajectory::LinearTrajectory(uint32_t t_0, float x0, float y0, float xdest, float ydest)
{
    this->t_0 = t_0;

    V = 0.1;
    vd = 0.0;

    d_current = 0.0;

    Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    
    theta0 = atan2(ydest - y0, xdest - x0);  // returned
}

Position2D LinearTrajectory::getPointAtTime(uint32_t current_time)  //TODO general class for this 
{

    d_current = V * (current_time - t_0);

    s = d_current / Dtotale;

    if (s < 0.1) {

    }
    else if (s > 0.9) {

    }
    else if (s > 1) {

    }
    else {
        
    }
}

float* LinearTrajectory::getVelAndTheta(uint32_t current_time) {
    d_current = V*(current_time - t_0);

    if (d_current < D_RAMPE) {
        vd = V * d_current / D_RAMPE;
    }
    else if (d_current > Dtotale - D_RAMPE) {
        vd = V * (Dtotale - d_current) / D_RAMPE;
    }
    else {
        vd = V;
    }

    float res[2] = {vd, theta0};

    return res;
}