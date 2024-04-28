/*
   
*/

#include "Trajectories/BezierTrajectory.hpp"

#include <Arduino.h>

#include "ROS.hpp"
#include "main_loop.hpp"


BezierTrajectory::BezierTrajectory(float initialGoalSpeed, float initialAccelParam, uint8_t nbPoints)
{
    goalSpeed = initialGoalSpeed;
    accelParam = initialAccelParam;
    this->nbPoints = nbPoints;

    xdest = 0.0; ydest = 0.0;

    trajectoryType = TrajectoryType::TRAJ_BEZIER;


    const std::array<typename bezier::Vec2, 3> controlPoints = {{
        {0.0, 0.0},
        {1.0, 2.0},
        {3.0, 4.0}
    }};
    
    const bezier::Point pointsArray[4] = {
        bezier::Point(0.0, 0.0),
        bezier::Point(1.0, 1.0),
        bezier::Point(2.0, 2.0),
        bezier::Point(3.0, 3.0)
    };

    std::vector<bezier::Point> pointsArray2(nbPoints);
    const bezier::Point test_point;
    // pointsArray2.push_back(test_point);

    pointsArray2[0].set(2.0, 3.0);

    // Convert the array into a vector
    std::vector<bezier::Point> pointsVector(std::begin(pointsArray), std::end(pointsArray));

    // MyClassBezier myBezier(controlPoints);

    bezier::Bezier<4> myBezier(pointsArray2);

    // Initialize the Bezier object with the vector
    // bezier::Bezier bezierCurve(pointsVector);
    // std::vector<Point>

    // switch (nbPoints)
    // {
    //     case 2:
    //         bezier::Bezier<2> m_bezier_2({ {120, 160}, {35, 200}, {220, 260} });
    //         break;

    //     case 3:
    //         bezier::Bezier<3> m_bezier_3({ {120, 160}, {35, 200}, {220, 260}, {220, 40} });
    //         break;
    // }
    
    // m_bezier_3.order;  // order of the curve so size - 1
    // m_bezier_3.size;  // number of points used

    std::array<bezier::Point, 4> points;
    std::array<bezier::Point, 3> bezierArray;

    // bezier::Bezier(bezierArray) bezier_test;


    // &&points

}

// A faire après avoir set la position du robot
void BezierTrajectory::setDest(Position2D orderInfo) {
    this->xdest = orderInfo.x;
    this->ydest = orderInfo.y;

    // Dtotale = sqrt((x0 - xdest) * (x0 - xdest) + (y0 - ydest) * (y0 - ydest));
    Dtotale = (float) m_bezier_3.length();  // length of the curve
    // theta0 = atan2(ydest - y0, xdest - x0);  // plus exact que le theta du robot

}



void BezierTrajectory::updateTrajectoryState() {

    x = m_bezier_3.valueAt(s, 0);
    y = m_bezier_3.valueAt(s, 0);
    // theta = m_bezier_3.tangentAt(s);//TODO

    V = currentSpeed;
    omega = 0.0;

    // Update des vitesses absolues du goal point en linéaire
    ppoint_d[0] = V*cos(theta); //TODO
    ppoint_d[1] = V*sin(theta); //TODO
}


