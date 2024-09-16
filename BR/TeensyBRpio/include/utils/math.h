#ifndef _MATH_EXT_H
#define _MATH_EXT_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#ifndef PI
#define PI M_PI 
#endif

#define __constrain(x, a, b) std::min(std::max(x, a), b)

inline float constrain(float x, float a, float b) {
    return __constrain(x, a, b);
}
inline double constrain(double x, float a, float b) {
    return __constrain(x, (double)a, (double)b);
}

#define cos(a) std::cos(a)
#define sin(a) std::sin(a)

#define isnan(a) std::isnan(a)
#define isinf(a) std::isinf(a)

#endif

#endif