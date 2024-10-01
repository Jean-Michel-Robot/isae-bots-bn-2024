#include "feedback/PositionEstimatorOdo.hpp"

#include <cmath>

template <OdoEncoder T, typename TMethod>
PositionEstimatorOdo<T, TMethod>::PositionEstimatorOdo(T encoder, double_t ticksPerRad, double_t ticksPerMillimeter, double_t correctionFactorLR)
    : m_ticksPerRad(ticksPerRad), m_ticksPerMillimeter(ticksPerMillimeter), m_correctionFactorLR(correctionFactorLR), m_encoder(std::move(encoder)),
      m_filter(1e-2), m_position(), m_odoLeftCount(0), m_odoRightCount(0), m_positionThetaOdo(0), m_positionThetaOffset(0) {}

template <OdoEncoder T, typename TMethod>
void PositionEstimatorOdo<T, TMethod>::resetPosition(Position2D<Millimeter> pos) {
    m_position = pos;
    m_positionThetaOffset += pos.theta - Angle(m_positionThetaOdo);
    m_positionThetaOdo = ((double_t)m_odoRightCount * m_correctionFactorLR - (double_t)m_odoLeftCount) / m_ticksPerRad + m_positionThetaOffset;
}

template <OdoEncoder T, typename TMethod>
Position2D<Meter> PositionEstimatorOdo<T, TMethod>::getRobotPosition() const {
    return m_position.toMeters();
}

template <OdoEncoder T, typename TMethod>
int32_t PositionEstimatorOdo<T, TMethod>::getLeftOdoCount() const {
    return m_odoLeftCount;
}

template <OdoEncoder T, typename TMethod>
int32_t PositionEstimatorOdo<T, TMethod>::getRightOdoCount() const {
    return m_odoRightCount;
}

template <OdoEncoder T, typename TMethod>
void PositionEstimatorOdo<T, TMethod>::update(double_t interval) {
    int32_t deltaL, deltaR;

    deltaL = -m_odoLeftCount;
    deltaR = -m_odoRightCount;

    m_odoLeftCount = m_encoder.getLeftCounter();
    m_odoRightCount = m_encoder.getRightCounter();

    deltaL += m_odoLeftCount;
    deltaR += m_odoRightCount;

    /*
    TODO: Le '> 0' est à modifier ?
        Pour trouver un juste milieu entre :
        update regulierement (pour limiter les erreurs de approx. geometrique de petits deplacements)
        et avoir des nombres "grand" (et limiter les erreurs de calculs de mantis...),

        suite a calculs, sur les dimensions de la table l'erreur de mantis est faible (10e-9mm par ticks au max)
     */
    if (deltaL != 0 || deltaR != 0) {
        double_t old_positionTheta = m_positionThetaOdo;
        m_positionThetaOdo = ((double_t)m_odoRightCount * m_correctionFactorLR - (double_t)m_odoLeftCount) / m_ticksPerRad + m_positionThetaOffset;

        double_t R = ((double_t)deltaR * m_correctionFactorLR + (double_t)deltaL) / 2. / m_ticksPerMillimeter;

        // Dans le repere local du roobot (x etant devant)
        double_t dx = R;
        double_t dy = R;

        TMethod::template applyMethod<T>(*this, dx, dy, deltaL, deltaR);

        m_position.x += std::cos(old_positionTheta) * dx + std::sin(old_positionTheta) * dy;
        m_position.y += std::sin(old_positionTheta) * dx - std::cos(old_positionTheta) * dy;
    }

    m_filter.update(m_positionThetaOdo, interval);
    m_position.theta = m_filter.value();
}

template <OdoEncoder T>
void MethodMoveFirst::applyMethod(const PositionEstimatorOdo<T, MethodMoveFirst> &estimator, double_t &dx, double_t &dy, double_t deltaL,
                                  double_t deltaR) {
    dx *= 1.0;
    dy *= 0.0;
}

template <OdoEncoder T>
void MethodUpdateThetaFirst::applyMethod(const PositionEstimatorOdo<T, MethodUpdateThetaFirst> &estimator, double_t &dx, double_t &dy,
                                         double_t deltaL, double_t deltaR) {
    double_t deltaTheta = ((double_t)deltaR * estimator.m_correctionFactorLR - (double_t)deltaL) / estimator.m_ticksPerRad;
    dx *= std::cos(deltaTheta);
    dy *= std::sin(deltaTheta);
}

template <OdoEncoder T>
void LegacyMethod3::applyMethod(const PositionEstimatorOdo<T, LegacyMethod3> &estimator, double_t &dx, double_t &dy, double_t deltaL,
                                double_t deltaR) {
    double_t deltaTheta = (deltaR * estimator.m_correctionFactorLR - deltaL) / estimator.m_ticksPerRad;
    if (deltaTheta != 0) {
        dx *= std::sin(deltaTheta) / deltaTheta;
        dy *= (1 - std::cos(deltaTheta)) / deltaTheta;
    } else {
        dx *= 1;
        dy *= 0;
    }
}

// Explicit instantiation of the estimator
// Template classes need either to have all their implementation in the .hpp file or to be explicitly instantiated for the particular types they are
// used with.
#include "specializations/feedback.hpp"
#ifdef _ODOS
template class PositionEstimatorOdo<encoders_t, ODOS_METHOD>;
#endif