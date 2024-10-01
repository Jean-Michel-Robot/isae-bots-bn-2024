#ifndef _POSITION_ESTIMATOR_ODO_HPP_
#define _POSITION_ESTIMATOR_ODO_HPP_

#include "configuration.hpp"

#include "encoders/OdoEncoder.hpp"
#include "geometry/Position2D.hpp"
#include "math/LowPassFilter.hpp"

class MethodMoveFirst;
class MethodUpdateThetaFirst;
class LegacyMethod3;

/**
 * Estimates the position of a two-wheel robot based on two odometers. This class satisfies concept PositionFeedback.
 *
 * @tparam TMethod Either MethodMoveFirst, MethodUpdateThetaFirst or LegacyMethod3.
 */
template <OdoEncoder T, typename TMethod>
class PositionEstimatorOdo {
  public:
    PositionEstimatorOdo(T encoder, double_t ticksPerRad, double_t ticksPerMillimeter, double_t correctionFactorLR);

    PositionEstimatorOdo()
        requires Default<T>
        : PositionEstimatorOdo(T(), ECARTS_ODOS, UNITS_ODOS, L_R_ODOS) {}

    /**
     * Read the counters from the encoder and update the estimated position.
     * @param interval The time elapsed since the last call to update. Must be strictly positive.
     */
    void update(double_t interval);
    void resetPosition(Position2D<Millimeter> pos);

    Position2D<Meter> getRobotPosition() const;

    int32_t getLeftOdoCount() const;
    int32_t getRightOdoCount() const;

  private:
    friend class MethodMoveFirst;
    friend class MethodUpdateThetaFirst;
    friend class LegacyMethod3;

    double_t m_ticksPerRad;
    double_t m_ticksPerMillimeter;
    double_t m_correctionFactorLR;

    T m_encoder;
    LowPassFilter<double_t> m_filter;

    Position2D<Millimeter> m_position;

    int32_t m_odoLeftCount;
    int32_t m_odoRightCount;

    double_t m_positionThetaOdo;
    double_t m_positionThetaOffset;
};

/// Update x and y based on the current orientation, then update theta
class MethodMoveFirst {
    template <OdoEncoder T, typename TMethod>
    friend class PositionEstimatorOdo;

    template <OdoEncoder T>
    static void applyMethod(const PositionEstimatorOdo<T, MethodMoveFirst> &estimator, double_t &dx, double_t &dy, double_t deltaL, double_t deltaR);
};

/// Update theta then infer the linear displacement
class MethodUpdateThetaFirst {
    template <OdoEncoder T, typename TMethod>
    friend class PositionEstimatorOdo;

    template <OdoEncoder T>
    static void applyMethod(const PositionEstimatorOdo<T, MethodUpdateThetaFirst> &estimator, double_t &dx, double_t &dy, double_t deltaL,
                            double_t deltaR);
};

/// Ask Etienne Arlaud
class [[deprecated("Copied from old code base; not documented")]] LegacyMethod3 {
    template <OdoEncoder T, typename TMethod>
    friend class PositionEstimatorOdo;

    template <OdoEncoder T>
    static void applyMethod(const PositionEstimatorOdo<T, LegacyMethod3> &estimator, double_t &dx, double_t &dy, double_t deltaL, double_t deltaR);
};

#endif