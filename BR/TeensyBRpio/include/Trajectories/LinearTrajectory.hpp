#ifndef H_LINEAR_TRAJECTORY
#define H_LINEAR_TRAJECTORY

#include "Trajectories/Trajectory.hpp"

class LinearTrajectory : public Trajectory
{
public :

    LinearTrajectory(float initialGoalSpeed, float initialAccelParam);
    // TODO destructeur

    void setDest(OrderType order) override;


private:

    // variables caractéristiques de la trajectoire
    // pour le déplacement linéaire c'est juste la position de fin
    float xdest, ydest;

    bool detectEndRamp() override;
    void updateTrajectoryState() override;
    

};


#endif