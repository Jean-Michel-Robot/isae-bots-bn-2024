#include "Position2D.h"
#include <cmath>

#include "Arduino.h"

template<typename Unit>
Position2D<Unit>::Position2D(float x, float y, float theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

template<typename Unit>
Position2D<Unit>::Position2D( Position2D<Unit> const& pos)
{
    x = pos.x;
    y = pos.y;
    theta = pos.theta;
}

Position2D<Meter> convert( Position2D<Millimeter> const& pos) {
    auto converted = pos / 1000.0;
    return Position2D<Meter>(converted.x, converted.y, converted.theta);
}

Position2D<Millimeter> convert( Position2D<Meter> const& pos) {
    auto converted = pos * 1000.0;
    return Position2D<Millimeter>(converted.x, converted.y, converted.theta);
}

template<typename Unit>
void Position2D<Unit>::changeReferentiel(Position2D<Unit> const &ref)
{
    Position2D<Unit> positionInRefWithoutRotation = *this - ref;
    x = positionInRefWithoutRotation.x * cos(ref.theta) + positionInRefWithoutRotation.y *sin(ref.theta) ;
    y = -positionInRefWithoutRotation.x * sin(ref.theta) + positionInRefWithoutRotation.y * cos(ref.theta) ;
    theta -= ref.theta;
}

template<typename Unit>
void Position2D<Unit>::operator=(const Position2D<Unit> &pos)
{
    x = pos.x;
    y = pos.y;
    theta = pos.theta;
}

#if defined(ARDUINO) or defined(__SIMU__)
template<typename Unit>
String Position2D<Unit>::toString() const
{
    return String("(") + String(x) + "," + String(y) + ";" + String(theta) + ")";
}
#else
template<typename Unit>
std::string Position2D<Unit>::to_string()const
{
    return "("+ std::to_string(x) + "," + std::to_string(y) + ";" + std::to_string(theta) + ")";
}
#endif

template<typename Unit>
void Position2D<Unit>::operator+=(const Position2D<Unit> &pos)
{
    this->x += pos.x;
    this->y += pos.y;
    this->theta += pos.theta;
}

template<typename Unit>
void Position2D<Unit>::operator-=(const Position2D<Unit> &pos)
{
    this->x -= pos.x;
    this->y -= pos.y;
    this->theta -= pos.theta;
}

template<typename Unit>
void Position2D<Unit>::operator*=(float factor)
{
    this->x *= factor;
    this->y *= factor;
}

template<typename Unit>
void Position2D<Unit>::operator/=(float factor)
{
    this->x /= factor;
    this->y /= factor;
}

template<typename Unit>
float Position2D<Unit>::s_dist(Position2D<Unit> const& a, Position2D<Unit> const& b)
{
    return sqrt((a.x - b.x)*(a.x-b.x) + (a.y - b.y)*(a.y-b.y));
}

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

template<typename Unit>
float Position2D<Unit>::s_angleBetweenTwoPoints(Position2D<Unit> const& a,Position2D<Unit> const& b) // calcul l'angle pour le vecteur AB
{
    if(s_isStrictEgalityXY(a,b))
        return 0; // on évite les nan
    return atan2(b.y-a.y,b.x-a.x);
}

template class Position2D<Meter>;
template class Position2D<Millimeter>;
