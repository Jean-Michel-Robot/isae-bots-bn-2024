#ifndef _SPEC_MANAGER_HPP_
#define _SPEC_MANAGER_HPP_

#include "specializations/actuators.hpp"
#include "specializations/clock.hpp"
#include "specializations/controller.hpp"
#include "specializations/feedback.hpp"

#include "manager/ControllerManager.hpp"

using manager_t = manager::ControllerManager<actuators_t, controller_t, feedback_t, _clock_t>;

#endif