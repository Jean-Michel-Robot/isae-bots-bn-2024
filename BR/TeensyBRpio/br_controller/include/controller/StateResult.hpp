#ifndef _CONTROLLER_STATE_RESULT_HPP_
#define _CONTROLLER_STATE_RESULT_HPP_

#include "controller/DisplacementKind.hpp"
#include "geometry/Angle.hpp"
#include "trajectories/Trajectory.hpp"

#include <cstdint>
#include <memory>
#include <numbers>
#include <optional>
#include <variant>

namespace controller {

class TrajectoryContainer {
  public:
    /// @param trajectory must not be null
    TrajectoryContainer(std::unique_ptr<Trajectory> trajectory, DisplacementKind kind, std::optional<Angle> finalOrientation)
        : trajectory(std::move(trajectory)), kind(kind), finalOrientation(finalOrientation) {}

    std::unique_ptr<Trajectory> trajectory;
    DisplacementKind kind;
    std::optional<Angle> finalOrientation;
};

class Ongoing {};

class InitialRotationComplete : public TrajectoryContainer {
  public:
    using TrajectoryContainer::TrajectoryContainer;
};

class TrajectoryComplete {
  public:
    TrajectoryComplete(DisplacementKind kind, std::optional<Angle> finalOrientation) : kind(kind), finalOrientation(finalOrientation) {}

    DisplacementKind kind;
    std::optional<Angle> finalOrientation;
};

class BadRobotOrientation : public TrajectoryContainer {
  public:
    using TrajectoryContainer::TrajectoryContainer;
};

class FinalRotationComplete {};

class BrakingComplete {
  public:
    BrakingComplete() : suspendedTrajectory(std::nullopt) {};
    BrakingComplete(TrajectoryContainer suspendedTrajectory) : suspendedTrajectory(std::move(suspendedTrajectory)) {};

    std::optional<TrajectoryContainer> suspendedTrajectory;
};

class RotationComplete {};

using StateUpdateResult =
    std::variant<Ongoing, InitialRotationComplete, TrajectoryComplete, BadRobotOrientation, FinalRotationComplete, BrakingComplete, RotationComplete>;

class UpdateResultCode {
  public:
    enum Flag : uint8_t {
        /// End of order (end of displacement without final orientation or end of final rotation)
        TERMINAL = 1,
        TRAJECTORY_COMPLETE = 2,
        ROTATION_COMPLETE = 4,
        WAS_REVERSE = 8,
    };
    constexpr UpdateResultCode() = default;
    constexpr UpdateResultCode(uint8_t value) : m_value(value) {}
    constexpr operator uint8_t() const { return m_value; }

    static constexpr uint8_t STOPPED = TERMINAL;

    static constexpr uint8_t INITIAL_ROTATION_COMPLETE = ROTATION_COMPLETE;
    static constexpr uint8_t FINAL_ROTATION_COMPLETE = ROTATION_COMPLETE | TERMINAL;

    static constexpr uint8_t FORWARD_TRAJECTORY_COMPLETE = TRAJECTORY_COMPLETE;
    static constexpr uint8_t REVERSE_TRAJECTORY_COMPLETE = TRAJECTORY_COMPLETE | WAS_REVERSE;

    static constexpr uint8_t ARRIVED_FORWARD = FORWARD_TRAJECTORY_COMPLETE | TERMINAL;
    static constexpr uint8_t ARRIVED_REVERSE = REVERSE_TRAJECTORY_COMPLETE | TERMINAL;

  private:
    uint8_t m_value;
};

} // namespace controller

#endif