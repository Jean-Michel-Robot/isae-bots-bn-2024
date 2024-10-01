#ifndef _CONTROLLER_EVENT_HPP_
#define _CONTROLLER_EVENT_HPP_

#include "geometry/Speeds.hpp"

#include <variant>

namespace controller {

class MaxSpeedsChanged {
  public:
    MaxSpeedsChanged(Speeds newSpeeds) : newSpeeds(newSpeeds) {}
    Speeds newSpeeds;
};

using ControllerEvent = std::variant<MaxSpeedsChanged>;

} // namespace controller

#endif