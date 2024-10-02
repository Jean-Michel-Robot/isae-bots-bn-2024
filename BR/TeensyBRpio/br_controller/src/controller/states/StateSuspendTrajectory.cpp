#include "controller/states/StateSuspendTrajectory.hpp"
#ifdef _DEBUG
#include "logging.hpp"
#endif

namespace controller {

StateSuspendTrajectory::StateSuspendTrajectory(Speeds robotSpeed, Accelerations brakingDecelerations,
                                               TrajectoryContainer suspendedTrajectory)
    : StateBraking(robotSpeed, brakingDecelerations), m_suspendedTrajectory(std::move(suspendedTrajectory)) {}

ControllerStatus StateSuspendTrajectory::getStatus() const {
    return SuspendingTrajectory;
}

StateUpdateResult StateSuspendTrajectory::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
#ifdef _DEBUG
    if (!m_suspendedTrajectory.trajectory) {
        abort("Illegal call to StateSuspendTrajectory::update after m_suspendedTrajectory was moved out.");
    }
#endif

    StateUpdateResult result = StateBraking::update(interval, setpoint, actualRobotPosition);
    if (std::holds_alternative<BrakingComplete>(result)) {
        return BrakingComplete(std::move(m_suspendedTrajectory));
    } else {
        return result;
    }
}

} // namespace controller