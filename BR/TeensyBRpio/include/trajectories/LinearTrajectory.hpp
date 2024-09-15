#ifndef H_LINEAR_TRAJECTORY
#define H_LINEAR_TRAJECTORY

#include "trajectories/Trajectory.hpp"
#include "geometry/Position2D.h"

class LinearTrajectory : public Trajectory
{
public :

    LinearTrajectory(float initialGoalSpeed=0.25 * MAX_LINEAR_GOAL_SPEED, float initialAccelParam=DEFAULT_LINEAR_ACCEL_PARAM);
    // TODO destructeur

    void setDest(Position2D<Meter> orderInfo) override;


private:

    // variables caractéristiques de la trajectoire
    // pour le déplacement linéaire c'est juste la position de fin
    float xdest, ydest;

    void updateTrajectoryState() override;
    

};


#endif