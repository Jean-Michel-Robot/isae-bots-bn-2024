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


/*
 * Point/position 2D avec methodes de calcul géométrique
 * peut correspondre à un point (x,y) ou une position de robot (x,y,theta)
 * */

class Position2D
{
public :

    Position2D(float x=0.0,float y=0.0,float theta=0.0);
    Position2D(Position2D const& pos); // constructeur de copie
    void changeReferentiel(Position2D const& ref);
    float x = 0,y = 0,theta = 0; // j'ai mis x,y,theta en public pour faciliter l'utilisation de la classe
    void operator+=(Position2D const& pos);
    void operator-=(Position2D const& pos);
    void operator*=(float factor); // doesn't modify theta, only a scale operation
    void operator/=(float factor); // idem
    void operator=(Position2D const& pos); // operateur de copie
    float norm(){return s_dist(*this,Position2D{0.0,0.0,0.0});}

#if defined(ARDUINO) or defined(__SIMU__)
    String toString() const;
    
#else
    std::string to_string()const;
#endif
    static float s_dist(Position2D const& a,Position2D const& b); // distance euclienne
    static float s_angleBetweenTwoPoints(Position2D const& a,Position2D const& b); // calcul l'angle pour le vecteur AB
    static bool s_isStrictEgalityXY(Position2D const& a,Position2D const& b){return a.x == b.x && a.y == b.y;}
};

Position2D operator+(Position2D const& posa, Position2D const& posb);
Position2D operator-(Position2D const& posa, Position2D const& posb);
Position2D operator*(Position2D const& pos, float factor); // doesn't modify theta, only a scale operation
Position2D operator/(Position2D const& pos, float factor); // idem

#endif
