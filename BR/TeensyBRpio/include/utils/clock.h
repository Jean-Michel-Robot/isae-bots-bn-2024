#ifndef _CLOCK_FN_H
#define _CLOCK_FN_H

#ifdef ARDUINO
// Arduino defines micros and millis.
#include <Arduino.h>
#else
#include <chrono>
#define micros() std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
#define millis() std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()

#endif

#endif