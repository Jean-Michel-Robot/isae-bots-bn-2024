#include "controller/UnicycleController.hpp"
#include "controller/states/StateBraking.hpp"
#include "controller/states/StateFinalRotation.hpp"
#include "controller/states/StateInitialRotation.hpp"
#include "controller/states/StateStandStill.hpp"
#include "controller/states/StateSuspendTrajectory.hpp"
#include "controller/states/StateTrajectoryWithRamp.hpp"
#include "controller/states/StateUninitialized.hpp"

#include "logging.hpp"
#include "rotations/SetHeadingProfile.hpp"
#include <cmath>
#include <numbers>

template <class... Ts>
struct overload : Ts... {
    using Ts::operator()...;
};

namespace controller {

template <ErrorConverter TConverter>
UnicycleController<TConverter>::UnicycleController(Vector2D<Meter> trackingOffset, TConverter converter, Accelerations brakeAccelerations,
                                                   Speeds maxSpeeds, Accelerations maxAccelerations)
    : m_offset(trackingOffset), m_brakeAccelerations(brakeAccelerations), m_maxSpeeds(maxSpeeds), m_maxAccelerations(maxAccelerations), m_setpoint(),
      m_goalPointSpeed(), m_actualSpeed(), m_estimatedRelSpeed(), m_converter(std::move(converter)), m_event() {
    setCurrentState<StateUninitialized>();
}

template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::updateCommand(double_t interval, Position2D<Meter> robotPosition) {
    m_event = UpdateResultCode();

    // Update estimated speed
    m_actualSpeed.update(robotPosition, interval);
    m_estimatedRelSpeed.linear = robotPosition.makeRelative(m_actualSpeed).x;
    m_estimatedRelSpeed.angular = m_actualSpeed.value().theta;

    // Update setpoint
    StateUpdateResult result = this->getCurrentState().update(interval, m_setpoint, robotPosition);

    // Visit result (make appropriate state transitions)
    std::visit( //
        overload{
            [](Ongoing &r) {},
            [&](InitialRotationComplete &r) {
                m_event = UpdateResultCode::INITIAL_ROTATION_COMPLETE;
                setCurrentState<StateTrajectoryWithRamp>(std::move(r.trajectory), m_maxSpeeds.linear, m_maxAccelerations.linear, r.kind,
                                                         r.finalOrientation);
            },
            [&](TrajectoryComplete &r) {
                if (r.finalOrientation) {
                    m_event = UpdateResultCode::TRAJECTORY_COMPLETE;
                    startRotation<SetHeadingProfile>(m_setpoint.theta, *r.finalOrientation);
                } else {
                    m_event = UpdateResultCode::ARRIVED_FORWARD;
                    setCurrentState<StateStandStill>(m_setpoint, robotPosition, /* preserveCurrentSetpoint = */ true);
                }
                m_event = m_event | (UpdateResultCode::WAS_REVERSE * uint32_t(r.kind == REVERSE));
            },
            [&](BadRobotOrientation &r) {
                m_setpoint.theta = robotPosition.theta;
                setCurrentState<StateSuspendTrajectory>(m_estimatedRelSpeed, m_brakeAccelerations, std::move(r));
            },
            [&](FinalRotationComplete &r) {
                m_event = UpdateResultCode::FINAL_ROTATION_COMPLETE;
                makeStill(robotPosition);
            },
            [&](BrakingComplete &r) {
                if (r.suspendedTrajectory) {
                    startTrajectory(std::move(r.suspendedTrajectory->trajectory), r.suspendedTrajectory->kind,
                                    r.suspendedTrajectory->finalOrientation);
                } else {
                    m_event = UpdateResultCode::STOPPED;
                    makeStill(robotPosition);
                }
            },
            [](RotationComplete &r) { log(ERROR, "StateResult 'RotationComplete' should not be returned to the controller"); } //
        },
        result);

    // Apply tracking offset
    Vector2D<Meter> trackingPoint = applyOffset(m_setpoint);
    Vector2D<Meter> actualPosition = applyOffset(robotPosition);

    m_goalPointSpeed.update(trackingPoint, interval);

    // Compute error
    m_converter.update(trackingPoint - actualPosition, interval);
    Vector2D<Meter> convertedError = m_goalPointSpeed.value() + m_converter.value();

    // Compute command from error
    double_t alpha = m_offset.x;
    double_t beta = m_offset.y;
    double_t theta = robotPosition.theta;

    double_t cmd_v = ((alpha * std::cos(theta) - beta * std::sin(theta)) * convertedError.x +
                      (alpha * std::sin(theta) + beta * std::cos(theta)) * convertedError.y) /
                     alpha;
    double_t cmd_omega = (-std::sin(theta) * convertedError.x + std::cos(theta) * convertedError.y) / alpha;

#ifdef _DEBUG
    m_lastCmd = Speeds(cmd_v, cmd_omega);
#endif

    return Speeds(cmd_v, cmd_omega);
}

// Orders

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setSetpoint(Position2D<Meter> setpoint) {
    m_setpoint = setpoint;
}

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::brakeToStop() {
    if ((getStatus() & 0b1110) != 0) {
        setCurrentState<StateBraking>(m_estimatedRelSpeed, m_brakeAccelerations);
    }
}

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::startTrajectory(DisplacementKind kind, std::unique_ptr<Trajectory> trajectory,
                                                     std::optional<Angle> finalOrientation) {
    startTrajectory(std::move(trajectory), kind, finalOrientation);
}

// Private
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::startTrajectory(std::unique_ptr<Trajectory> trajectory, DisplacementKind kind,
                                                     std::optional<Angle> finalOrientation) {
    if (!trajectory) {
        log(ERROR, "startTrajectory() called with a null trajectory.");
        return;
    }
    setCurrentState<StateInitialRotation>(std::move(trajectory), m_setpoint, m_maxSpeeds.angular, m_maxAccelerations.angular, kind, finalOrientation);
}

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::startRotation(std::unique_ptr<OrientationProfile> rotation) {
    setCurrentState<StateFinalRotation>(std::move(rotation), m_maxSpeeds.angular, m_maxAccelerations.angular);
}

// Private
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::makeStill(Position2D<Meter> robotPosition) {
    setCurrentState<StateStandStill>(m_setpoint, robotPosition);
}

template <ErrorConverter TConverter>
void UnicycleController<TConverter>::reset(Position2D<Meter> robotPosition) {
    ControllerStatus status = getStatus();
    if (status != Still && status != Invalid) {
        log(WARN, "Controller reset while the robot was moving.");
    }

    m_setpoint = robotPosition;
    m_goalPointSpeed.reset(applyOffset(robotPosition));
    m_actualSpeed.reset(robotPosition);
    m_converter.reset();
    makeStill(robotPosition);
}

// Getters and setters
template <ErrorConverter TConverter>
ControllerStatus UnicycleController<TConverter>::getStatus() const {
    return this->getCurrentState().getStatus();
}

template <ErrorConverter TConverter>
UpdateResultCode UnicycleController<TConverter>::getLastEvent() const {
    return m_event;
}

template <ErrorConverter TConverter>
Vector2D<Meter> UnicycleController<TConverter>::getTrackingOffset() const {
    return m_offset;
}

template <ErrorConverter TConverter>
Position2D<Meter> UnicycleController<TConverter>::getSetpoint() const {
    return m_setpoint;
}

template <ErrorConverter TConverter>
Vector2D<Meter> UnicycleController<TConverter>::getGoalPoint() const {
    return applyOffset(m_setpoint);
}

template <ErrorConverter TConverter>
Vector2D<Meter> UnicycleController<TConverter>::getGoalPointSpeed() const {
    return m_goalPointSpeed;
}

template <ErrorConverter TConverter>
const TConverter &UnicycleController<TConverter>::getErrorConverter() const {
    return m_converter;
}
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setErrorConverter(TConverter converter) {
    m_converter = std::move(converter);
}

template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::getBrakeAccelerations() const {
    return m_brakeAccelerations;
}
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setBrakeAccelerations(Speeds brakeAccelerations) {
    m_brakeAccelerations = brakeAccelerations;
}

template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::getMaxSpeeds() const {
    return m_maxSpeeds;
}
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setMaxSpeeds(Speeds speeds, bool persist) {
    if (persist) {
        m_maxSpeeds = speeds;
    }
    getCurrentState().notify(MaxSpeedsChanged(speeds));
}

template <ErrorConverter TConverter>
Accelerations UnicycleController<TConverter>::getMaxAccelerations() const {
    return m_maxAccelerations;
}
template <ErrorConverter TConverter>
void UnicycleController<TConverter>::setMaxAccelerations(Accelerations accelerations) {
    m_maxAccelerations = accelerations;
}

// Private
template <ErrorConverter TConverter>
Vector2D<Meter> UnicycleController<TConverter>::applyOffset(Position2D<Meter> position) const {
    return position.relativeOffset(m_offset.x, m_offset.y);
}

#ifdef _DEBUG
template <ErrorConverter TConverter>
Speeds UnicycleController<TConverter>::getLastCommand() const {
    return m_lastCmd;
}
#endif

} // namespace controller

// Explicit instantiation of the controller
// Template classes need either to have all their implementation in the .hpp file or to be explicitly instantiated for the particular types they are
// used with.
#include "specializations/controller.hpp"
template class controller::UnicycleController<converter_t>;
