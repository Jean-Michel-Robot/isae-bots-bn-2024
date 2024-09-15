#ifndef __H_POSITION2D
#define __H_POSITION2D

#ifdef ARDUINO
    #include <Arduino.h>
    typedef String string_t;   
#else
    #include <string>
    typedef std::string string_t;
#endif

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

    string_t to_string() const;

    static float s_dist(Position2D<Unit> const& a,Position2D<Unit> const& b); // distance euclienne
    static float s_angleBetweenTwoPoints(Position2D<Unit> const& a,Position2D<Unit> const& b); // calcul l'angle pour le vecteur AB
    static bool s_isStrictEgalityXY(Position2D<Unit> const& a,Position2D<Unit> const& b){return a.x == b.x && a.y == b.y;}

};

template<typename Unit>
Position2D<Unit> operator+(const Position2D<Unit> &posa, const Position2D<Unit> &posb)
{
    return Position2D<Unit>(posa.x+posb.x,posa.y+posb.y,posa.theta+posb.theta);
}

template<typename Unit>
Position2D<Unit> operator-(const Position2D<Unit> &posa, const Position2D<Unit> &posb)
{
    return Position2D<Unit>(posa.x-posb.x,posa.y-posb.y,posa.theta-posb.theta);
}

template<typename Unit>
Position2D<Unit> operator*(const Position2D<Unit> &pos, float factor)
{
    return Position2D<Unit>(pos.x*factor,pos.y*factor,pos.theta);
}

template<typename Unit>
Position2D<Unit> operator/(const Position2D<Unit> &pos, float factor)
{
    return Position2D<Unit>(pos.x/factor,pos.y/factor,pos.theta);
}

#endif

Position2D<Millimeter> convert(Position2D<Meter> const& a);
Position2D<Meter> convert(Position2D<Millimeter> const& a);