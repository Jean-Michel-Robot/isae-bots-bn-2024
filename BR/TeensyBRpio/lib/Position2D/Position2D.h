#ifndef __H_POSITION2D
#define __H_POSITION2D
#ifdef __linux__

// #include <string>
// include a rajouter pour compiler en C++ sur un PC
#include <cmath>
#ifndef __SIMU__
// #define min(A,B) std::min(A,B)
// #define sq(A) ((A)*(A))
#else
#include "../../../Simulation/Arduino_defines.h"
#endif
#else
#include <Arduino.h>
#endif

#include <Arduino.h>

class Meter;
class Millimeter;

/*
 * Point/position 2D avec methodes de calcul géométrique
 * peut correspondre à un point (x,y) ou une position de robot (x,y,theta)
 * */

template<typename Unit>
class Position2D
{
public:

    Position2D(float x=0.0,float y=0.0,float theta=0.0);
    Position2D(Position2D<Unit> const& pos); // constructeur de copie
    void changeReferentiel(Position2D<Unit> const& ref);
    float x = 0,y = 0,theta = 0; // j'ai mis x,y,theta en public pour faciliter l'utilisation de la classe
    void operator+=(Position2D<Unit> const& pos);
    void operator-=(Position2D<Unit> const& pos);
    void operator*=(float factor); // doesn't modify theta, only a scale operation
    void operator/=(float factor); // idem
    void operator=(Position2D<Unit> const& pos); // operateur de copie
    float norm() const {return s_dist(*this,Position2D{0.0,0.0,0.0});}

#if defined(ARDUINO) or defined(__SIMU__)
    String toString() const;
    
#else
    std::string to_string()const;
#endif
    static float s_dist(Position2D<Unit> const& a,Position2D<Unit> const& b); // distance euclienne
    static float s_angleBetweenTwoPoints(Position2D<Unit> const& a,Position2D<Unit> const& b); // calcul l'angle pour le vecteur AB
    static bool s_isStrictEgalityXY(Position2D<Unit> const& a,Position2D<Unit> const& b){return a.x == b.x && a.y == b.y;}

};

template<typename Unit>
Position2D<Unit> operator+(Position2D<Unit> const& posa, Position2D<Unit> const& posb);
template<typename Unit>
Position2D<Unit> operator-(Position2D<Unit> const& posa, Position2D<Unit> const& posb);
template<typename Unit>
Position2D<Unit> operator*(Position2D<Unit> const& pos, float factor); // doesn't modify theta, only a scale operation
template<typename Unit>
Position2D<Unit> operator/(Position2D<Unit> const& pos, float factor); // idem

#endif

Position2D<Millimeter> convert(Position2D<Meter> const& a);
Position2D<Meter> convert(Position2D<Millimeter> const& a);