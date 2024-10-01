#include "manager/ManagerState.hpp"
#include "geometry/Position2D.hpp"
#include "logging.hpp"

namespace manager {

template <Actuators TActuators, typename TController>
ManagerState<TActuators, TController>::ManagerState() {}

template <Actuators TActuators, typename TController>
bool ManagerState<TActuators, TController>::sendOrder(TController &controller, Position2D<Meter> robotPosition,
                                                      std::function<void(TController &, Position2D<Meter>)> order) {
    return false;
}

} // namespace manager

#include "specializations/actuators.hpp"
#include "specializations/controller.hpp"
template class manager::ManagerState<actuators_t, controller_t>;