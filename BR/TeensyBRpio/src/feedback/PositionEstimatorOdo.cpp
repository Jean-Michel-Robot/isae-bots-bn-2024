#include "feedback/PositionEstimatorOdo.hpp"

#include "utils/FilterLowPass.h"
#include "defines.hpp"
#include "motors.hpp"
#include "utils/clock.h"
#include "ros/ROS.hpp"

#define METHOD 1 // choix de la méthode d'approximation

#include <cmath>

PositionEstimatorOdo::PositionEstimatorOdo(OdoEncoder &encoder): m_encoder(encoder)
{
    m_encoder.resetCounters();

    m_odoLeftCount = 0;
    m_odoRightCount = 0;

    m_position.x = 0;
    m_position.y = 0;
    m_positionThetaOffset = 0;
    m_positionThetaOdo = 0;
}

void PositionEstimatorOdo::resetPosition(Position2D<Millimeter> pos)
{
    PositionFeedback::resetPosition(pos);
    m_positionThetaOffset += pos.theta - m_positionThetaOdo;
    m_positionThetaOdo = (double(m_odoRightCount) * L_R_ODOS - double(m_odoLeftCount)) / ECARTS_ODOS + m_positionThetaOffset;
}

void PositionEstimatorOdo::update()
{
    int32_t deltaL, deltaR;

    deltaL = -m_odoLeftCount;
    deltaR = -m_odoRightCount;

    m_odoLeftCount = m_encoder.getLeftCounter();
    m_odoRightCount = m_encoder.getRightCounter();

    deltaL += m_odoLeftCount;
    deltaR += m_odoRightCount;
    /*
     Le '> 0' est à modifier ?
     Pour trouver un juste milieu entre :
     update regulierement (pour limiter les erreurs de approx. geometrique de petits deplacements)
     et avoir des nombres "grand" (et limiter les erreurs de calculs de mantis...),

     //suite a calculs, sur les dimensions de la table l'erreur de mantis est faible (10e-9mm par ticks au max)
     // coucou Etienne comment va-tu :p
  */
    if (abs(deltaL) + abs(deltaR) > 0)
    { // si un tick est repéré sur un odomètre
        double old_positionTheta = m_positionThetaOdo;
        m_positionThetaOdo = (double(m_odoRightCount) * L_R_ODOS - double(m_odoLeftCount)) / ECARTS_ODOS + m_positionThetaOffset;

        double R = (double(deltaR) * L_R_ODOS + double(deltaL)) / 2. / UNITS_ODOS;

        //Dans le repere local du roobot (x etant devant)
        double dx = R;
        double dy = R;

#if METHOD == 1 // le robot s'est déplacé tout droit selon le précédent théta
        dx *= 1.0;
        dy *= 0.0;
#elif METHOD == 2 // le robot s'est déplacé tout droit selon le nouveau theta
        double deltaTheta = (double(deltaR) * L_R_ODOS - double(deltaL)) / ECART_ODOS;
        dx *= cos(deltaTheta);
        dy *= sin(deltaTheta);
#elif METHOD == 3 // demande à Etienne Arlaud
        double deltaTheta = (deltaR * L_R_ODOS - deltaL) / ECART_ODOS;
        if (deltaTheta != 0)
        {
            dx *= sin(deltaTheta) / deltaTheta;
            dy *= (1 - cos(deltaTheta)) / deltaTheta;
        }
        else
        {
            dx *= 1;
            dy *= 0;
        }
#else
#error method undefined
#endif
        //Changement de base dans le repere global
        m_position.x += cos(old_positionTheta) * dx + sin(old_positionTheta) * dy;
        m_position.y += sin(old_positionTheta) * dx - cos(old_positionTheta) * dy;
    }

    static FilterLowPass fil(1e-2);
    m_position.theta = fil.computeOutput(m_positionThetaOdo, micros());
}

constexpr int32_t PositionEstimatorOdo::getLeftOdoCount() const {
    return m_odoLeftCount;
}
constexpr int32_t PositionEstimatorOdo::getRightOdoCount() const {
    return m_odoRightCount;
}

void PositionEstimatorOdo::sendPosition() const {
    PositionFeedback::sendPosition();
    ROS::instance().sendOdosCounts(m_odoLeftCount, m_odoRightCount);
}

#ifndef __SIMU__

#ifdef ODO_HARD
#include "encoders/HardOdoEncoder.hpp"
HardOdoEncoder &encoder = HardOdoEncoder::instance();
#elif defined(ODO_SOFT)
#include "encoders/SoftOdoEncoder.hpp"
SoftOdoEncoder encoder;
#else
#error Odemeters method undefined
#endif

PositionFeedback &PositionFeedback::instance() {
    static PositionEstimatorOdo estimator(encoder);
    return estimator;
}

#endif