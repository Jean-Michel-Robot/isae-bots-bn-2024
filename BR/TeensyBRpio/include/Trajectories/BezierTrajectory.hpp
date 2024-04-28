#ifndef H_LINEAR_TRAJECTORY
#define H_LINEAR_TRAJECTORY

#include <array>
#include <vector>

#include "Trajectories/Trajectory.hpp"
#include "bezier/bezier.hpp"


// Assuming you have a class where you want to initialize a bezier::Bezier attribute
class MyClassBezier {
public:
    // Constructor that takes an array of control points
    template<size_t N>
    MyClassBezier(const std::array<typename bezier::Vec2, N+1>& controlPoints) {
        bezierCurve = bezier::Bezier<N>(controlPoints);
    }

    // Setter method to initialize the bezier curve with an array of control points
    template<size_t N>
    void setBezierCurve(const std::array<typename bezier::Vec2, N+1>& controlPoints) {
        bezierCurve = bezier::Bezier<N>(controlPoints);
    }

private:
    bezier::Bezier<bezier::Vec2::size> bezierCurve;
};

class MyClassBezier2 {
public:
    // Constructor that takes an array of control points
    template<size_t N>
    MyClassBezier2(std::vector<bezier::Point> controlPoints) {
        m_bezier = bezier::Bezier<N>(controlPoints);
    }

private:
    bezier::Bezier<bezier::Vec2::size> m_bezier;
};


class BezierTrajectory : public Trajectory
{
public :

    BezierTrajectory(float initialGoalSpeed, float initialAccelParam, uint8_t nbPoints, std::vector<int>* controlPoints);
    // TODO destructeur

    void setDest(Position2D orderInfo) override;


private:

    // variables caractéristiques de la trajectoire
    // pour une courbe de bezier c'est un vecteur de n points
    static uint8_t nbPoints;
    float xdest, ydest;

    // bezier::Point** p_point;
    // std::array<bezier::Point, 3> bezierArray;

    const std::vector<bezier::Point> vec_test = {};
    // { {120, 160}, {35, 200}, {220, 260}, {220, 40} }

    bezier::Bezier<2> m_bezier_2;
    bezier::Bezier<3> m_bezier_3;
    bezier::Bezier<4> m_bezier_4;

    // std::array<bezier::Vec2, nbPoints> array;



    void updateTrajectoryState() override;
    
};


#endif