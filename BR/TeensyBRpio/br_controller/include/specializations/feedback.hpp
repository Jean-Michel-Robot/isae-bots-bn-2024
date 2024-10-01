#ifndef _SPEC_FEEDBACK_HPP_
#define _SPEC_FEEDBACK_HPP_

#if _SIMULATION

#include "feedback/UnicycleStateSimulator.hpp"

using feedback_t = UnicycleStateSimulator;

#elif defined(ARDUINO)

#include "configuration.hpp"
#include "encoders/QuadDecode.h"
#include "feedback/PositionEstimatorOdo.hpp"

#define _ODOS

using encoders_t = QuadDecodeRef;
using feedback_t = PositionEstimatorOdo<encoders_t, ODOS_METHOD>;

#else
#error "Set _SIMULATION to 1 or add an implementation of PositionFeedback for the current platform."
#endif

#endif
