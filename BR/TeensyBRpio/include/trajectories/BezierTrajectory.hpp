#ifndef H_LINEAR_TRAJECTORY
#define H_LINEAR_TRAJECTORY

#include "trajectories/Trajectory.hpp"
#include "geometry/bezier.hpp"
#include "geometry/Position2D.h"

class BezierTrajectory : public Trajectory
{
public :

    BezierTrajectory(float initialGoalSpeed, float initialAccelParam);
    // TODO destructeur

    void setDest(Position2D<Meter> orderInfo) override;


private:

    // variables caractéristiques de la trajectoire
    // pour une courbe de bezier c'est un vecteur de n points
    uint8_t nbPoints;
    float xdest, ydest;

    void updateTrajectoryState() override;
    

};


#endif