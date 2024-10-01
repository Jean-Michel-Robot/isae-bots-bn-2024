#include "trajectories/CompoundTrajectory.hpp"
#include "logging.hpp"

CompoundTrajectory::CompoundTrajectory(std::unique_ptr<Trajectory> first, std::unique_ptr<Trajectory> second)
    : m_first(std::move(first)), m_second(std::move(second)) {
    if (!m_first && m_second) {
        m_first = std::move(m_second);
    }
    if (!m_first) {
        abort("At least one part of a CompoundTrajectories must be non-null.");
    }
}

bool CompoundTrajectory::advance(double_t distance) {
    if (m_first->advance(distance)) {
        return true;
    } else {
        if (m_second) {
#ifdef _DEBUG
            if (m_first->getCurrentPosition() != m_second->getCurrentPosition()) {
                log(WARN, "Non contiguous CompoundTrajectory detected");
            }
#endif
            m_first = std::move(m_second);
            return m_first->advance(distance);
        } else {
            return false;
        }
    }
}

Position2D<Meter> CompoundTrajectory::getCurrentPosition() const {
    return m_first->getCurrentPosition();
}

std::optional<double_t> CompoundTrajectory::getRemainingDistance() const {
    std::optional<double_t> firstDist = m_first->getRemainingDistance();
    if (firstDist) {
        if (m_second) {
            std::optional<double_t> secondDist = m_second->getRemainingDistance();
            if (secondDist) {
                return *firstDist + *secondDist;
            }
        } else {
            return *firstDist;
        }
    }
    return std::nullopt;
}
