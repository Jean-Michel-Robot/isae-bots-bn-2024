#ifndef _ROS_POSITION_ORDER_HPP_
#define _ROS_POSITION_ORDER_HPP_

#include "Actuators.hpp"
#include "Clock.hpp"
#include "PositionFeedback.hpp"
#include "controller/UnicycleController.hpp"
#include "geometry/Position2D.hpp"
#include "manager/ControllerManager.hpp"
#include "math/ProportionalIntegralDerivative.hpp"

enum GoalType {
    UNVALID_GOALTYPE = -1,
    /// Linear displacement with final orientation
    LINEAR_FINAL = 0,
    /// Linear displacement without final orientation
    LINEAR_TRANS = 1,
    /// Rotation without displacement
    ORIENTATION = 9,
    /// Linear displacement backwards
    LINEAR_REVERSE = 8,

    /// Emergency braking
    STOP = 2,
    /// Resets the estimated position
    RESET = 3
};

class DisplacementOrder {
  public:
    using controller_t = controller::UnicycleController<ProportionalIntegralDerivative<Vector2D<Meter>>>;
    template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
    using manager_t = manager::ControllerManager<TActuators, controller_t, TFeedback, TClock>;

    DisplacementOrder(GoalType type, Position2D<Millimeter> goalPosition);
    DisplacementOrder(int type, Position2D<Millimeter> goalPosition);

    template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
    void operator()(manager_t<TActuators, TFeedback, TClock> &manager) const;

    GoalType type;
    Position2D<Millimeter> position;

  private:
    /// Sends the order to the manager or the controller depending on the goal type.
    template <Actuators TActuators, PositionFeedback TFeedback, Clock TClock>
    static void goTo(manager_t<TActuators, TFeedback, TClock> &manager, DisplacementKind kind, Vector2D<Millimeter> goalPosition,
                     std::optional<Angle> angle);
};

#endif