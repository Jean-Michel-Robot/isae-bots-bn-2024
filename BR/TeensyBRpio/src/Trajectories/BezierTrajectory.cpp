/*
   
*/

#include "Trajectories/BezierTrajectory.hpp"

#include <Arduino.h>

#include "ROS.hpp"
#include "main_loop.hpp"

#define LENGTH_NB_INTERVALS 100


BezierTrajectory::BezierTrajectory(float initialGoalSpeed, float initialAccelParam, uint8_t nbPoints, std::vector<int>* controlPoints)
{
    goalSpeed = initialGoalSpeed;
    accelParam = initialAccelParam;
    this->nbPoints = nbPoints;

    xdest = 0.0; ydest = 0.0;

    trajectoryType = TrajectoryType::TRAJ_BEZIER;

    std::vector<bezier::Point> pointsArray(nbPoints);

    // const std::array<bezier::Vec2, 3> controlPoints2 = {{
    //     {0.0, 0.0},
    //     {1.0, 2.0},
    //     {3.0, 4.0}
    // }};


    for (size_t i=0; i<controlPoints->size(); i++) {
        pointsArray[i].set(controlPoints[i][0], controlPoints[i][1]);
    }

    // bezier::Bezier<4> myBezier(pointsArray);

    switch (nbPoints) {
        case 2:
            m_bezier_2 = bezier::Bezier<2>(pointsArray);
            break;
        case 3:
            m_bezier_3 = bezier::Bezier<3>(pointsArray);
            break;
        case 4:
            m_bezier_4 = bezier::Bezier<4>(pointsArray);
            break;
    }

    // myClassBezier2->setBezierCurve(pointsArray);

    // std::vector<bezier::Point> pointsArray(nbPoints);

    // MyClassBezier myBezier(controlPoints2);

}

// A faire après avoir set la position du robot
void BezierTrajectory::setDest(Position2D orderInfo) {
    this->xdest = orderInfo.x;
    this->ydest = orderInfo.y;

    // Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    Dtotale = (float) m_bezier_3.length(LENGTH_NB_INTERVALS);  // length of the curve
    // theta0 = atan2(ydest - y0, xdest - x0);  // plus exact que le theta du robot

}



void BezierTrajectory::updateTrajectoryState() {

    x = m_bezier_3.valueAt(s, 0);
    y = m_bezier_3.valueAt(s, 0);
    theta = (float) m_bezier_3.tangentAt(s).angle();

    V = currentSpeed;
    omega = 0.0;

    // Update des vitesses absolues du goal point
    ppoint_d[0] = V*cos(theta);
    ppoint_d[1] = V*sin(theta);
}


