#ifndef _H_DEFINE
#define _H_DEFINE

#if defined(__SIMU__) & defined(ARDUINO)
#warning trying to use simu node on a real teensy card
#endif

#ifndef __linux__
#include <Arduino.h>
#undef abs
#elif defined(__SIMU__)
#include "../Simulation/Arduino_defines.h"
typedef bool boolean;
#endif
//code de l'asserv

#include <cmath>

// Macros
#define sign(value) (value > 0 ? 1 : -1)

// Parameters
#define ODRIVE_RX_PIN 0
#define ODRIVE_TX_PIN 1


#endif
