#include "motors/MotorStub2.hpp"

MotorStub2::MotorStub2(std::shared_ptr<Speeds> speedsPtr) : m_speeds(std::move(speedsPtr)), m_isReady(false) {}

void MotorStub2::sendCommand(Speeds speeds) {
    m_speeds->linear = speeds.linear;
    m_speeds->angular = speeds.angular;
}

void MotorStub2::switchOn() {
    m_isReady = true;
}

void MotorStub2::switchOff() {
    m_isReady = false;
}

void MotorStub2::update(double_t interval) {
    // Nothing to do
}

bool MotorStub2::isReady() const {
    return m_isReady;
}
bool MotorStub2::isIdle() const {
    return !m_isReady;
}
