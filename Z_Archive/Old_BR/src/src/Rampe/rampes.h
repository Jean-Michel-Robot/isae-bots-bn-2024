#ifndef __H_RAMPES
#define __H_RAMPES

#include "rampeGenerique.h"
#include "../Position2D/Position2D.h"
#ifdef __SIMU__
#include "../../a_geometric_tools.h"
#elif defined ARDUINO
#include "../../a_geometric_tools.h"
#else
#define PI M_PI
inline float modulo_x2(float value)
{
  return (value > PI ? value - 2 * PI : (value < -PI ? value + 2 * PI : value));
}
inline float modulo_pipi(float value)
{
    return modulo_x2(fmod(value,2*PI));
}
#endif
#include <cmath>
class RampePosition : public RampeGenerique
{
public :
    RampePosition(float acceleration, float deccelerationSpeedShift, float deccelerationArret, float speedGoal);
    void setParcours(Position2D posStart,Position2D posEnd,bool isFast,float startSpeed = 0.0,float endSpeed = 0.0); // on donne le parcours a effectuer
    void setParcours(Position2D posStart, Position2D posEnd, RampeGenerique::DynamicType dynamicType, float startSpeed, float endSpeed);
    Position2D calcPos(float time) const; // calcul de la position de la rampe lineaire
    Position2D getObjective()const{return m_posEnd;}
    float getThetaParcours() const {return Position2D::s_angleBetweenTwoPoints(m_posStart,m_posEnd);;}
    void triggerBrake(float time);
private :
    Position2D m_posStart;
    Position2D m_posEnd;
};

class RampeOrientation : public RampeGenerique
{
public :
    RampeOrientation(float acceleration, float decelerationSpeedShift, float decelerationStop, float speedGoal);
    void setParcours(float thetaStart,float thetaEnd,bool isFast, float startSpeed = 0.0, float endSpeed = 0.0);
    void setParcoursByShortestPath(float thetaStart,float thetaEnd,bool isFast, float startSpeed = 0.0, float endSpeed = 0.0);
    float calcOrientation(float time) const;
    float getObjective()const {return m_thetaEnd;}
    bool isRotationTrigo()const {return modulo_pipi(m_thetaEnd - m_thetaStart) > 0;}
    float getTravelLength()const {return std::abs(modulo_pipi(m_thetaEnd - m_thetaStart));}

private :
    float m_thetaStart;
    float m_thetaEnd;
 };

class RampeVitesseOnly
{
public :
  RampeVitesseOnly(float acceleration,float speedMax);
  void startParcours(float time);
  float calcSpeed(float time) const;
private :
  float m_timeStartRecal = 0.0;
  float m_acc;
  float m_speedMax;
};

#endif
