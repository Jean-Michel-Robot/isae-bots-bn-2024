#ifndef __H_GLOBAL_VARIABLE
#define __H_GLOBAL_VARIABLE

#include "defines.hpp"
#include "geometry/Position2D.h"
#include "utils/math.h"
#include "utils/clock.h"

// l'utilisation des fonctions en constexpr permet au compilateur de remplacer textuellement une fonction basique et de l'utiliser pour les constantes constexpr
// avec le mot clef constexpr, le calcul est effectué directement à la compilation et pas à l'execution
// le mot clef inline effectue la meme chose mais ne fonctionne pas avec les constantes (mais a moins de restriction)

float modulo_x2(float value) ;//Ramene un angle € [-2*PI;2*PI] dans [-PI;PI]
float modulo_pipi(float value);//Ramene un angle € [-inf;inf] dans [-PI;PI]
float modulo_pi2pi2(float value);//Ramene un angle € [-inf;inf] dans [-PI/2;PI/2]
float constexpr degToRad(float value) { return value * PI / 180;}
float constexpr radToDeg(float value) { return value * 180 / PI;}

// Helper function to switch to asserv point frame
template<typename Unit>
Position2D<Unit> toAsservPointFrame(Position2D<Unit> pos) {
  return Position2D<Unit>(
    pos.x + ASSERV_ALPHA*cos(pos.theta),
    pos.y + ASSERV_ALPHA*sin(pos.theta),
    pos.theta
  );
  //TODO assuming ASSERV_BETA is 0 here, could generalize with ASSERV_BETA as well
}

inline float timeFloat(){return float(micros())*1e-6;} // retourne le temps micros() sous un format float (en secondes)
#endif
