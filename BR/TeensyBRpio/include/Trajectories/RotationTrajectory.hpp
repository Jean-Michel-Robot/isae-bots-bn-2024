#ifndef H_ROTATION_TRAJECTORY
#define H_ROTATION_TRAJECTORY

#include "Trajectories/Trajectory.hpp"

class RotationTrajectory : public Trajectory
{
public :

    RotationTrajectory(float initialGoalSpeed, float initialAccelParam);
    // TODO destructeur

    void setDest(Position2D orderInfo) override;

private:

    // variables caractéristiques de la trajectoire
    // pour le déplacement linéaire c'est juste la position de fin
    float thetaDest;

    void updateTrajectoryState() override;
    
};


#endif