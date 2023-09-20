#include "a_define.h"
#include "a_geometric_tools.h"

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

float modulo_2pi_0(float value){
  float tmp = modulo_pipi(value);
  if(tmp < -PI/4) return tmp + 2*PI;
  else return tmp;
}

float modulo_0_2pi(float value){
  float tmp = modulo_pipi(value);
  if(tmp > PI/4) return tmp - 2*PI;
  else return tmp;
}
