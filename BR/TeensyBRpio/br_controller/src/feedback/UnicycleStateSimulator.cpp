#include "feedback/UnicycleStateSimulator.hpp"

#include "Clock.hpp"
#include <cmath>

UnicycleStateSimulator::UnicycleStateSimulator(double_t noise_stddev) : m_speeds(std::make_shared<Speeds>()), m_position(), m_noise(noise_stddev) {}

void UnicycleStateSimulator::setSpeeds(Speeds speeds) {
    m_speeds->linear = speeds.linear;
    m_speeds->angular = speeds.angular;
}

void UnicycleStateSimulator::resetPosition(Position2D<Millimeter> pos) {
    m_position = pos.toMeters();
}

void UnicycleStateSimulator::update(double_t interval) {
    double_t linOffset = (m_speeds->linear + m_noise()) * interval;

    m_position.x += std::cos(m_position.theta) * linOffset;
    m_position.y += std::sin(m_position.theta) * linOffset;
    m_position.theta += Angle((m_speeds->angular + m_noise()) * interval);
}

Position2D<Meter> UnicycleStateSimulator::getRobotPosition() const {
    return m_position;
}

MotorStub2 UnicycleStateSimulator::createMotorStub() const {
    return MotorStub2(m_speeds);
}

UnicycleStateSimulator::Noise::Noise(double_t standardDeviation) {
    if (standardDeviation > 0) {
        m_noise = std::normal_distribution<double_t>(0, standardDeviation);
    }
}

double_t UnicycleStateSimulator::Noise::operator()() {
    static std::default_random_engine generator(SystemClock().micros());
    if (m_noise) {
        return (*m_noise)(generator);
    } else {
        return 0;
    }
}
