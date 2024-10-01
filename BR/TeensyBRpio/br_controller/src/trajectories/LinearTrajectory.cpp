#include "trajectories/LinearTrajectory.hpp"
#include <algorithm>

LinearTrajectory::LinearTrajectory(Vector2D<Meter> origin, Vector2D<Meter> destination)
    : m_origin(origin), m_destination(destination), m_direction((m_destination - m_origin).normalize()),
      m_totalLength(Vector2D<Meter>::distance(m_origin, m_destination)), m_position(0) {}

bool LinearTrajectory::advance(double_t distance) {
    if (m_position == m_totalLength) {
        return false;
    }

    m_position += distance;
    if (m_position > m_totalLength) {
        m_position = m_totalLength;
    }
    return true;
}

Position2D<Meter> LinearTrajectory::getCurrentPosition() const {
    return Position2D<Meter>(m_origin + m_direction * m_position, m_direction.argument());
}

std::optional<double_t> LinearTrajectory::getRemainingDistance() const {
    return std::make_optional<double_t>(m_totalLength - m_position);
}
