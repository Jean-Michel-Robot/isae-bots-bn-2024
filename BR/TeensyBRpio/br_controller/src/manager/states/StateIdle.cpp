#include "manager/states/StateIdle.hpp"
#include "logging.hpp"

namespace manager {

template <Actuators TActuators, typename TController>
StateIdle<TActuators, TController>::StateIdle() {
    log(INFO, "Entering manager state: Idle");
}

template <Actuators TActuators, typename TController>
ManagerStatus StateIdle<TActuators, TController>::getStatus() const {
    return Idle;
}

template <Actuators TActuators, typename TController>
ManagerStatus StateIdle<TActuators, TController>::update(TActuators &actuators) {
    // TODO: check actuators state?
    return Idle;
}

} // namespace manager

#include "specializations/actuators.hpp"
#include "specializations/controller.hpp"
template class manager::StateIdle<actuators_t, controller_t>;
