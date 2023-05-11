#include "GeometricTools.hpp"

#include "defines.hpp"
#include <Position2D.h>

//Ramene un angle € [-2*PI;2*PI] dans [-PI;PI]
float modulo_x2(float value)
{
  return (value > PI ? value - 2 * PI : (value < -PI ? value + 2 * PI : value));
}
// ramene un angle € [-inf,inf] dans [-PI/2,PI/2]
float modulo_pi2pi2(float value)
{
  float val_pipi = modulo_pipi(value);
  if (val_pipi > PI / 2) return val_pipi - PI;
  if (val_pipi < - PI / 2) return val_pipi + PI;
  return val_pipi;
}

float modulo_pipi(float value)//ramene un angle € R dans [-PI;PI]
{
    return modulo_x2(fmod(value,2*PI));
}


// Helper function to switch to asserv point frame
Position2D toAsservPointFrame(Position2D pos) {
  return Position2D(
    pos.x + ASSERV_ALPHA*cos(pos.theta),
    pos.x + ASSERV_ALPHA*sin(pos.theta),
    pos.theta
  );
  //TODO assuming ASSERV_BETA is 0 here, could generalize with ASSERV_BETA as well
}