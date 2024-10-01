#ifndef _MANAGER_STATE_ACTIVE_HPP_
#define _MANAGER_STATE_ACTIVE_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

template <Actuators TActuators, typename TController>
class StateActive : public ManagerState<TActuators, TController> {
  public:
    StateActive();
    ManagerStatus getStatus() const override;
    ManagerStatus update(TActuators &actuators) override;
    bool sendOrder(TController &controller, Position2D<Meter> robotPosition, std::function<void(TController &, Position2D<Meter>)> order) override;
};

} // namespace manager

#endif