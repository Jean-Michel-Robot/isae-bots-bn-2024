#include "rotations/SetHeadingProfile.hpp"
#include "defines/math.hpp"
#include <cmath>

SetHeadingProfile::SetHeadingProfile(Angle initialHeading, Angle targetHeading) : m_currentHeading(initialHeading), m_targetHeading(targetHeading) {}

bool SetHeadingProfile::advance(double_t diffAngle) {
    if (m_currentHeading == m_targetHeading) {
        return false;
    }
    Angle newHeading = m_currentHeading + Angle(diffAngle * sign(m_targetHeading - m_currentHeading));
    if (sign(m_targetHeading - m_currentHeading) != sign(m_targetHeading - newHeading)) {
        m_currentHeading = m_targetHeading;
    } else {
        m_currentHeading = newHeading;
    }
    return true;
}

Angle SetHeadingProfile::getCurrentOrientation() const {
    return m_currentHeading;
}

std::optional<double_t> SetHeadingProfile::getRemainingAngle() const {
    return abs(m_targetHeading - m_currentHeading);
}