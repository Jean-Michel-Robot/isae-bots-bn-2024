#ifndef _SPEC_CONTROLLER_HPP_
#define _SPEC_CONTROLLER_HPP_

#include "controller/UnicycleController.hpp"
#include "math/ProportionalIntegralDerivative.hpp"

using converter_t = ProportionalIntegralDerivative<Vector2D<Meter>>;
using controller_t = controller::UnicycleController<converter_t>;

#endif
