#ifndef __H_LINEAR_TRAJECTORY
#define __H_LINEAR_TRAJECTORY

// #include <Arduino.h>
// // #include "a_define.h"
// #include <Position2D.h>
// #include <Ramp.hpp>
#include <Trajectory.hpp>


class LinearTrajectory : Trajectory 
{
public :
    LinearTrajectory();  // quatre coords définissant le segment + l'origine de temps
    // TODO destructeur

    void setDest(float x0, float y0, float xdest, float ydest);

    // bool detectEndRamp();

    // Position2D getPointAtTime(uint32_t current_time);
    // Position2D calculateTrajCoords(float s);



private:

    float x0, y0, xdest, ydest;
    float theta0;
    float Dtotale;

};

#endif
