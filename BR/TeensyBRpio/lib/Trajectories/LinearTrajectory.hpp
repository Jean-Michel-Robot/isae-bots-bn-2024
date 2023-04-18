#ifndef __H_LINEAR_TRAJECTORY_2
#define __H_LINEAR_TRAJECTORY_2

#include <Trajectory.hpp>

class LinearTrajectory : Trajectory
{
public :

    LinearTrajectory(float initialGoalSpeed, float initialAccelParam);
    // TODO destructeur

    void setDest(float xdest, float ydest) override;

private:

    // variables caractéristiques de la trajectoire
    // pour le déplacement linéaire c'est juste la position de fin
    float xdest, ydest;

    bool detectEndRamp() override;
    void modifyVars() override;

};


#endif