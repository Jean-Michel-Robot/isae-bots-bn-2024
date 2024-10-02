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

class ManualSpeedCommand {
  public:
    ManualSpeedCommand(Speeds speeds) : speeds(speeds) {}
    Speeds speeds;
};

using ControllerEvent = std::variant<MaxSpeedsChanged, ManualSpeedCommand>;

} // namespace controller

#endif