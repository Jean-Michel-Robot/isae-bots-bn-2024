#ifndef _CONTROLLER_UNICYCLE_HPP_
#define _CONTROLLER_UNICYCLE_HPP_

#include "configuration.hpp"

#include "controller/ControllerState.hpp"
#include "defines/constraint.hpp"
#include "fsm/StateMachine.hpp"
#include "geometry/Speeds.hpp"
#include "math/Derivative.hpp"

#include <concepts>
#include <memory>
#include <optional>

template <typename T>
concept ErrorConverter = requires(T a, Vector2D<Meter> error, double_t interval) {
    a.update(error, interval);
    a.reset();
    { a.value() } -> std::convertible_to<Vector2D<Meter>>;
};

class OrientationProfile;

namespace controller {

class StateUninitialized;
class StateStandStill;
class StateInitialRotation;
class StateTrajectoryWithRamp;
class StateFinalRotation;
class StateSuspendTrajectory;
class StateBraking;

/**
 * A controller for a unicycle, nonholonomic robot. A robot is said to be unicycle if its displacement can be described only by its linear and angular
 * speeds. Such robots are usually driven by two differential wheels, but this controller makes no assumption as to the actuators of the robot.
 *
 * The controller is not connected to the motors. The command must be sent manually (see also ControllerManager).
 *
 * @tparam TConverter Error converter (such as a PID). How the error is converted is up to the converter. See also concept ErrorConverter and
 * constructor parameter "converter"
 */
template <ErrorConverter TConverter>
class UnicycleController : private fsm::StateMachine<ControllerState> {
  public:
    /**
     * Creates a controller. The controller must then be initialized with method reset() before it is used.
     *
     * @param trackingOffset The position of the tracking point, relative to the center of the robot. Its x coordinate must be strictly positive.
     * Using an off-center tracking point causes the point to move when the robot rotates, which allows to replace the angular speed with a linear
     * speed and to reduce the number of control variables to 2 instead of 3. The closer of the robot's center the tracking point is, the more dynamic
     * is the control, but the more instable it gets.
     *
     * @param converter Transforms the signed speed error between the tracking point of the robot and the goal point. The output of the converter is
     *added to the speed of the goal point to build the command. NB: The setpoint is the position where the center of the robot should be; the goal
     *point is the position where the tracking point of the robot should be (after applying the tracking offset).
     *
     * @param brakeAccelerations Maximal absolute accelerations when braking. Must be strictly positive.
     * @param maxSpeeds Maximal absolute speeds when moving or rotating. Both speeds must be strictly positive.
     * @param maxAccelerations Maximal accelerations/decelerations when starting and completing a trajectory or a rotation. Both accelerations must
     * be strictly positive.
     */
    UnicycleController(Vector2D<Meter> trackingOffset, TConverter converter, Accelerations brakeAccelerations, Speeds maxSpeeds,
                       Accelerations maxAccelerations);

    /// Initializes the controller with the default values from the configuration file.
    UnicycleController()
        requires Default<TConverter>
        : UnicycleController({ASSERV_ALPHA, ASSERV_BETA}, TConverter(), {BRAKING_LINEAR_ACCELERATION, BRAKING_ROTATION_ACCELERATION},
                             {MAX_LINEAR_GOAL_SPEED, MAX_ROTATION_GOAL_SPEED}, {DEFAULT_LINEAR_ACCELERATION, DEFAULT_ROTATION_ACCELERATION}) {}

    /**
     * Computes the suggested command to take the robot from its current position to the position of the setpoint.
     *
     * @param interval The time elapsed since the last update
     * @param robotPosition The actual position of the center of the robot
     */
    Speeds updateCommand(double_t interval, Position2D<Meter> robotPosition);

    // Orders

    /**
     * Moves the setpoint to the specified location. This bypasses the maximal speeds and accelerations.
     * If the new setpoint is too far from the previous setpoint or the robot position, the error and the command will skyrocket.
     *
     * The setpoint is the position where the center robot should **currently** be, not the position it should eventually reach.
     *
     * - To reset the controller if the robot was moved manually, use reset() instead.
     * - To request the robot to rotate or follow a trajectory, use startRotation() or startTrajectory() instead.
     */
    void setSetpoint(Position2D<Meter> setpoint);

    /**
     * Puts the controller in braking state. The robot will decelerate from its current estimated speed until it stops.
     * If there is an ongoing rotation or trajectory, it is cancelled.
     */
    void brakeToStop();

    /**
     * Starts a trajectory. If the robot is already moving, the previous trajectory or rotation is cancelled and the new trajectory starts
     * immediately.
     */
    void startTrajectory(DisplacementKind kind, std::unique_ptr<Trajectory> trajectory, std::optional<Angle> finalOrientation = std::nullopt);
    template <Derived<Trajectory> TTrajectory, typename... Args>
    void startTrajectory(DisplacementKind kind, Args &&...args, std::optional<Angle> finalOrientation) {
        startTrajectory(kind, std::make_unique<TTrajectory>(std::forward<Args>(args)...), finalOrientation);
    }

    /**
     * Starts a rotation. If the robot is already moving, the previous trajectory or rotation is cancelled and the rotation starts
     * immediately.
     */
    void startRotation(std::unique_ptr<OrientationProfile> rotation);
    template <Derived<OrientationProfile> TProfile, typename... Args>
    void startRotation(Args &&...args) {
        startRotation(std::make_unique<TProfile>(std::forward<Args>(args)...));
    }

    /**
     * Sets the setpoint to the actual robot position and resets the internal state of the controller, such as the error and the estimated speed of
     * the robot and the setpoint.
     */
    void reset(Position2D<Meter> robotPosition = {});

    // Getters and setters

    /// See ControllerStatus (in ControllerState.hpp)
    ControllerStatus getStatus() const;
    // Returns the event triggered by the last call to updateCommand. If no applicable event was triggered, this returns 0 instead.
    UpdateResultCode getLastEvent() const;

    /// See constructor documentation
    Vector2D<Meter> getTrackingOffset() const;
    /// The expected position of the center of the robot
    Position2D<Meter> getSetpoint() const;
    /// The goal point for the robot after applying the tracking offset to the setpoint
    Vector2D<Meter> getGoalPoint() const;
    Vector2D<Meter> getGoalPointSpeed() const;

    const TConverter &getErrorConverter() const;
    void setErrorConverter(TConverter converter);

    Speeds getBrakeAccelerations() const;
    void setBrakeAccelerations(Speeds brakeAccelerations);

    Speeds getMaxSpeeds() const;
    void setMaxSpeeds(Speeds speeds, bool persist = true);

    Accelerations getMaxAccelerations() const;
    void setMaxAccelerations(Accelerations speeds);

#ifdef _DEBUG
    Speeds getLastCommand() const;
#endif

  private:
    /// Convenience function to avoid writing "this->template setCurrentState" every time.
    template <Derived<ControllerState> TNewState, typename... Args>
    void setCurrentState(Args &&...args) {
        fsm::StateMachine<ControllerState>::template setCurrentState<TNewState>(std::forward<Args>(args)...);
    }

    void startTrajectory(std::unique_ptr<Trajectory> trajectory, DisplacementKind kind, std::optional<Angle> finalOrientation);
    /// Transit in state StandStill.
    void makeStill(Position2D<Meter> robotPosition);
    Vector2D<Meter> applyOffset(Position2D<Meter> position) const;

    Vector2D<Meter> m_offset;
    Accelerations m_brakeAccelerations;
    Speeds m_maxSpeeds;
    Accelerations m_maxAccelerations;

    Position2D<Meter> m_setpoint;
    Derivative<Vector2D<Meter>> m_goalPointSpeed;
    /// Estimated speed of the robot
    Derivative<Position2D<Meter>> m_actualSpeed;
    std::shared_ptr<Speeds> m_estimatedRelSpeed;

    TConverter m_converter;
    UpdateResultCode m_event;

#ifdef _DEBUG
    Speeds m_lastCmd;
#endif
};
} // namespace controller

#endif