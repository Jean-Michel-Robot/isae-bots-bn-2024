#include "math/Ramp.hpp"
#include "defines/math.hpp"
#include <cmath>

Ramp::Ramp(double_t targetSpeed, double_t maximalAcceleration, double_t currentSpeed)
    : m_isComplete(targetSpeed == currentSpeed), m_currentSpeed(currentSpeed), m_targetSpeed(targetSpeed), m_maxAcceleration(maximalAcceleration) {}

double_t Ramp::getCurrentSpeed() const {
    return m_currentSpeed;
}

double_t Ramp::getTargetSpeed() const {
    return m_targetSpeed;
}

double_t Ramp::getMaximalAcceleration() const {
    return m_maxAcceleration;
}

void Ramp::overwriteCurrentSpeed(double_t speed) {
    if (m_currentSpeed == speed) {
        return;
    }
    m_currentSpeed = speed;
    m_isComplete = false;
}

void Ramp::setTargetSpeed(double_t speed) {
    if (m_targetSpeed == speed) {
        return;
    }
    m_targetSpeed = speed;
    m_isComplete = false;
}

void Ramp::setMaximalAcceleration(double_t acceleration) {
    m_maxAcceleration = acceleration;
}

void Ramp::update(double_t interval) {
    if (!m_isComplete) {

        double_t maxDiff = m_maxAcceleration * interval;
        double_t requiredDiff = m_targetSpeed - m_currentSpeed;

        if (abs(requiredDiff) < maxDiff) {
            m_currentSpeed = m_targetSpeed;
            m_isComplete = true;
        } else {
            m_currentSpeed += maxDiff * sign(requiredDiff);
        }
    }
}

double_t Ramp::computeSpeedFromBrakingDistance(double_t distance) const {
    // Braking distance = v**2 / (2a)
    // It must be no greater than d = the remaining distance on the trajectory: v**2 / (2a) <= d
    // v <= sqrt(2ad)

    return std::sqrt(2 * m_maxAcceleration * distance);
}

void Ramp::ensureCanBrake(double_t distance) {
    setTargetSpeed(std::min(m_targetSpeed, computeSpeedFromBrakingDistance(distance)));
}
