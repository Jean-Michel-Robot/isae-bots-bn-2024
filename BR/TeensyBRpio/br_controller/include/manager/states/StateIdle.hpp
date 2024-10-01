#ifndef _MANAGER_STATE_IDLE_HPP_
#define _MANAGER_STATE_IDLE_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

template <Actuators TActuators, typename TController>
class StateIdle : public ManagerState<TActuators, TController> {
  public:
    StateIdle();
    ManagerStatus getStatus() const override;
    ManagerStatus update(TActuators &actuators) override;
};

} // namespace manager

#endif