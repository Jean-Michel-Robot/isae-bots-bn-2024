#ifndef _H_EST_ODO_
#define _H_EST_ODO_

#include "feedback/PositionFeedback.hpp"
#include "geometry/Position2D.h"
#include "encoders/OdoEncoder.hpp"
#include <stdint.h>

class PositionEstimatorOdo : public PositionFeedback
{
public:
    PositionEstimatorOdo(OdoEncoder &encoder);

    void resetPosition(Position2D<Millimeter> pos) override;
    constexpr int32_t getLeftOdoCount() const;
    constexpr int32_t getRightOdoCount() const;

    static constexpr float ECARTS_ODOS = 5980.73537126610L; // (ticks.rad^(-1) ecart entre les 2 odos
    static constexpr float UNITS_ODOS = 51.54179961710274L; // ticks.mm^(-1)
    static constexpr float L_R_ODOS = 1.0011809854125424L;  // rapport des rapports... (var en R et L)

protected:
    void update() override;
    void sendPosition() const override;

private:
    OdoEncoder &m_encoder;

    int32_t m_odoLeftCount;
    int32_t m_odoRightCount;

    float m_positionThetaOdo = 0.0;
    float m_positionThetaOffset;
};

#endif