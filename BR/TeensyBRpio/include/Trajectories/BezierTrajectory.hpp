#ifndef H_LINEAR_TRAJECTORY
#define H_LINEAR_TRAJECTORY

#include "Trajectories/Trajectory.hpp"
#include "bezier/bezier.hpp"

class BezierTrajectory : public Trajectory
{
public :

    BezierTrajectory(float initialGoalSpeed, float initialAccelParam);
    // TODO destructeur

    void setDest(Position2D orderInfo) override;


private:

    // variables caractéristiques de la trajectoire
    // pour une courbe de bezier c'est un vecteur de n points
    uint8_t nbPoints;
    float xdest, ydest;

    void updateTrajectoryState() override;
    

};


#endif