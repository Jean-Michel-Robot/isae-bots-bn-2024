#ifndef __H_LINEAR_TRAJECTORY_2
#define __H_LINEAR_TRAJECTORY_2

#include <Trajectory.hpp>

class LinearTrajectory2 : Trajectory
{
public :

    LinearTrajectory2(float initialGoalSpeed, float initialAccelParam);
    // TODO destructeur

    void setDest(float x0, float y0, float xdest, float ydest) override;

    bool detectEndRamp() override;

    void modifyVars(float *q) override;


private:

    // variables caractéristiques de la trajectoire

    float x0, y0, xdest, ydest;
    float theta0;
};


#endif