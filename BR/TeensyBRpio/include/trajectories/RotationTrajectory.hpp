#ifndef H_ROTATION_TRAJECTORY
#define H_ROTATION_TRAJECTORY

#include "trajectories/Trajectory.hpp"
#include "geometry/Position2D.h"

class RotationTrajectory : public Trajectory
{
public :

    RotationTrajectory(float initialGoalSpeed=0.25 * MAX_ROTATION_GOAL_SPEED, float initialAccelParam=DEFAULT_ROTATION_ACCEL_PARAM);
    // TODO destructeur

    void setDest(Position2D<Meter> orderInfo) override;

private:

    // variables caractéristiques de la trajectoire
    // pour le déplacement linéaire c'est juste la position de fin
    float thetaDest;

    void updateTrajectoryState() override;
    
};


#endif