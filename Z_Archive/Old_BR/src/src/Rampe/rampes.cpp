#include "rampes.h"
RampePosition::RampePosition(float acceleration, float deccelerationMax, float deccelerationArret, float speedGoal):
    RampeGenerique(acceleration,deccelerationMax,deccelerationArret,speedGoal)
{
    setDeccBrake(deccelerationArret);
}

void RampePosition::setParcours(Position2D posStart,Position2D posEnd, bool isFast,float startSpeed , float endSpeed )
{
    setParcours(posStart,posEnd,isFast?RampeGenerique::FAST:RampeGenerique::SMOOTH,startSpeed,endSpeed);
}

void RampePosition::setParcours(Position2D posStart, Position2D posEnd, RampeGenerique::DynamicType dynamicType, float startSpeed, float endSpeed)
{				// on donne le parcours a effectuer
    m_posStart = posStart;
    m_posEnd = posEnd;
    float length = Position2D::s_dist(posStart,posEnd);
    _setLinParcours(0.0,length,dynamicType,startSpeed,endSpeed);
}

Position2D RampePosition::calcPos(float time) const
{				// calcul de la position de la rampe (en distance)
    if (m_etat != ETAT_RUNNING )	// si la rampe n'est pas demarrée
        return Position2D(0.0,0.0,0.0);
    if(m_posLinFin == 0.0)
        return m_posEnd;
    return m_posStart + (m_posEnd - m_posStart) * _calcLinPos(time)/m_posLinFin;
}

void RampePosition::triggerBrake(float time)
{ // pour déclencher un freinage, on va modifier le point final de la rampe, et donc la recréer
    float currentSpeed = calcSpeed(time);
    Position2D rampePos = calcPos(time); // on va calculer à quel point le robot sera à vitesse 0, pour créer une rampe depuis le point actuel vers ce point d'arrêt
    float dist_frein = s_calcDistChgmtVitesse(currentSpeed, 0.0,m_accelerationHard,m_deccelerationBrake) + 0.1; // on ajoute 0.1mm pour amortir les incertitudes des float en calcul, et éviter de demander une rampe tout juste infaisable
    float thetaRampe = getThetaParcours();
    Position2D posStop = Position2D(dist_frein,0.0,0.0); // on a la distance, on la projette sur la rampe
    posStop.changeReferentiel(Position2D(0.0,0.0,-thetaRampe));
    posStop+=rampePos;
    setParcours(rampePos, posStop,RampeGenerique::BRAKE, currentSpeed, 0.0); // on recréé une rampe qui commence à vitesse non nulle
    startParcours(time);
}

RampeOrientation::RampeOrientation(float acceleration, float deccelerationSpeedShift, float deccelerationArret, float speedGoal):
    RampeGenerique(acceleration,deccelerationSpeedShift,deccelerationArret,speedGoal)
{
}

void RampeOrientation::setParcours(float thetaStart, float thetaEnd,bool isFast, float startSpeed , float endSpeed )
{				// on donne le parcours en angle a effectuer
    m_thetaStart = thetaStart;
    m_thetaEnd = thetaEnd;
    _setLinParcours(0.0,std::abs(thetaEnd - thetaStart),isFast?RampeGenerique::FAST:RampeGenerique::SMOOTH,startSpeed,endSpeed);
}

void RampeOrientation::setParcoursByShortestPath(float thetaStart, float thetaEnd, bool isFast, float startSpeed, float endSpeed)
{
    setParcours(thetaStart,thetaStart + modulo_pipi(thetaEnd-thetaStart),isFast,startSpeed,endSpeed); //both rotations are allowed
    //setParcours(thetaStart,thetaStart + modulo_2pi_0(thetaEnd-thetaStart),isFast,startSpeed,endSpeed); //always turn clockwise (or anticlockwise I dunno)
    //setParcours(thetaStart,thetaStart   + modulo_0_2pi(thetaEnd-thetaStart),isFast,startSpeed,endSpeed); //always turn anticlockwise (or clockwise I dunno)
}

float RampeOrientation::calcOrientation(float time) const
{
    if (m_etat != ETAT_RUNNING )	// si la rampe n'est pas demarrée
        return 0.0;
    if (m_posLinFin <= 0.0)
        return m_thetaEnd;
    return m_thetaStart + (m_thetaEnd - m_thetaStart) * _calcLinPos(time)/m_posLinFin;
}

RampeVitesseOnly::RampeVitesseOnly(float acceleration, float speedMax)
{
    m_acc = acceleration;
    m_speedMax = speedMax;
}

void RampeVitesseOnly::startParcours(float time)
{
    m_timeStartRecal = time;
}

float RampeVitesseOnly::calcSpeed(float time) const
{
    return constrain(m_acc * (time - m_timeStartRecal), float(0.0), m_speedMax);
}
