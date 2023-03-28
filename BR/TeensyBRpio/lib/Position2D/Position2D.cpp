#include "Position2D.h"
#include <cmath>

Position2D::Position2D(float x, float y, float theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

Position2D::Position2D( Position2D const& pos)
{
    x = pos.x;
    y = pos.y;
    theta = pos.theta;
}

void Position2D::changeReferentiel(Position2D const &ref)
{
    Position2D positionInRefWithoutRotation = *this - ref;
    x = positionInRefWithoutRotation.x * cos(ref.theta) + positionInRefWithoutRotation.y *sin(ref.theta) ;
    y = -positionInRefWithoutRotation.x * sin(ref.theta) + positionInRefWithoutRotation.y * cos(ref.theta) ;
    theta -= ref.theta;
}

void Position2D::operator=(const Position2D &pos)
{
    x = pos.x;
    y = pos.y;
    theta = pos.theta;
}

#if defined(ARDUINO) or defined(__SIMU__)
String Position2D::toString() const
{
    return String("(") + String(x) + "," + String(y) + ";" + String(theta) + ")";
}
#else
std::string Position2D::to_string()const
{
    return "("+ std::to_string(x) + "," + std::to_string(y) + ";" + std::to_string(theta) + ")";
}
#endif

void Position2D::operator+=(const Position2D &pos)
{
    this->x += pos.x;
    this->y += pos.y;
    this->theta += pos.theta;
}

void Position2D::operator-=(const Position2D &pos)
{
    this->x -= pos.x;
    this->y -= pos.y;
    this->theta -= pos.theta;
}

void Position2D::operator*=(float factor)
{
    this->x *= factor;
    this->y *= factor;
}

void Position2D::operator/=(float factor)
{
    this->x /= factor;
    this->y /= factor;
}
float Position2D::s_dist(Position2D const& a, Position2D const& b)
{
    return sqrt((a.x - b.x)*(a.x-b.x) + (a.y - b.y)*(a.y-b.y));
}

Position2D operator+(const Position2D &posa, const Position2D &posb)
{
    return Position2D(posa.x+posb.x,posa.y+posb.y,posa.theta+posb.theta);
}

Position2D operator-(const Position2D &posa, const Position2D &posb)
{
    return Position2D(posa.x-posb.x,posa.y-posb.y,posa.theta-posb.theta);
}

Position2D operator*(const Position2D &pos, float factor)
{
    return Position2D(pos.x*factor,pos.y*factor,pos.theta);
}

Position2D operator/(const Position2D &pos, float factor)
{
    return Position2D(pos.x/factor,pos.y/factor,pos.theta);
}

float Position2D::s_angleBetweenTwoPoints(Position2D const& a,Position2D const& b) // calcul l'angle pour le vecteur AB
{
    if(s_isStrictEgalityXY(a,b))
        return 0; // on évite les nan
    return atan2(b.y-a.y,b.x-a.x);
}
