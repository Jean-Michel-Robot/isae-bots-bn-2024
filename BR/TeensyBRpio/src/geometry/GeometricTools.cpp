#include "defines.hpp"
#include "geometry/Position2D.h"
#include "geometry/GeometricTools.hpp"

//Ramene un angle € [-2*PI;2*PI] dans [-PI;PI]
float modulo_x2(float value)
{
  return (value > PI ? value - 2 * PI : (value < -PI ? value + 2 * PI : value));
}
// ramene un angle € [-inf,inf] dans [-PI/2;PI/2]
float modulo_pi2pi2(float value)
{
  float val_pipi = modulo_pipi(value);
  if (val_pipi > PI / 2) return val_pipi - PI;
  if (val_pipi < - PI / 2) return val_pipi + PI;
  return val_pipi;
}

// ramene un angle € R dans [-PI;PI]
float modulo_pipi(float value)
{
    return modulo_x2(fmod(value,2*PI));
}

